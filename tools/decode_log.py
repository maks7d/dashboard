#!/usr/bin/env python3
"""Décodeur des logs binaires du dashboard (format défini dans Core/Inc/log_types.h).

Exemples :
    python tools/decode_log.py E:/DASH/S0042                 # toute une session, fusionnée par temps
    python tools/decode_log.py E:/DASH/S0042/SES_00.BIN  # un seul fichier
    python tools/decode_log.py E:/DASH/S0042 --csv sortie/   # un CSV par type d'événement
    python tools/decode_log.py E:/DASH/S0042 --stream session

Le temps affiché est "secondes depuis le démarrage" (tick FreeRTOS / 1000). Tous les fichiers d'une
mise sous tension partagent la même horloge : on peut donc fusionner IMU, CAN, tours et RPM.
"""
import argparse
import csv
import struct
import sys
from pathlib import Path

LOG_MAGIC = 0x474F4C44                       # "DLOG"
HEADER = struct.Struct('<IBBHIIHHIII')       # 32 octets, voir LogFileHeader_t
REC16 = struct.Struct('<IB11s')              # LogRecord_t
REC32 = struct.Struct('<IBB26s')             # LogRawRecord_t
STREAMS = {0: 'session', 1: 'raw', 2: 'sys'}
BACKENDS = {0: 'aucun', 1: 'SD', 2: 'flash'}
REASONS = ['boot', 'carte retirée', 'carte insérée', 'carte pleine', 'erreur SD',
           'erreur flash', 'flash reformatée', 'fichier brut plein']
# Bits de RCC->RSR (STM32H7, voir stm32h743xx.h : RCC_RSR_xxxRSTF_Pos)
RESET_FLAGS = [(1 << 21, 'BOR'), (1 << 22, 'PIN'), (1 << 23, 'POR'), (1 << 24, 'SW'),
               (1 << 26, 'IWDG'), (1 << 28, 'WWDG'), (1 << 30, 'LPWR')]


def decode_payload(rec_id, p):
    """Renvoie (nom, dict de champs) pour un identifiant d'événement."""
    if rec_id == 0x01:
        us, n = struct.unpack_from('<IH', p)
        return 'lap', {'lap_number': n, 'lap_s': us / 1e6}
    if rec_id == 0x02:
        return 'rpm', {'pulses_per_min': struct.unpack_from('<I', p)[0]}
    if rec_id == 0x03:
        return 'lux', {'lux': struct.unpack_from('<H', p)[0]}
    if rec_id == 0x04:
        return 'button', {'button': 'SW%d' % p[0]}
    if rec_id == 0x05:
        return 'water_temp', {'celsius': struct.unpack_from('<h', p)[0] / 100}
    if rec_id == 0x20:
        fw, rsr = struct.unpack_from('<II', p)
        flags = [n for m, n in RESET_FLAGS if rsr & m] or ['?']
        return 'boot', {'fw': '%d.%d.%d' % (fw >> 24, (fw >> 16) & 0xFF, fw & 0xFFFF), 'reset': '+'.join(flags)}
    if rec_id == 0x21:
        ds, dr, dy, heap, be, sd = struct.unpack_from('<HHHHBB', p)
        return 'stats', {'drop_session': ds, 'drop_raw': dr, 'drop_sys': dy, 'min_free_heap_kb': heap,
                         'backend': BACKENDS.get(be, be), 'sd_present': sd}
    if rec_id == 0x22:
        return 'power', {'event': {1: 'appui court', 2: 'extinction'}.get(p[0], p[0])}
    if rec_id == 0x23:
        be, why, free = struct.unpack_from('<BBI', p)
        return 'storage', {'backend': BACKENDS.get(be, be),
                           'reason': REASONS[why] if why < len(REASONS) else why, 'free_kb': free}
    if rec_id == 0x40:
        odr, *s = struct.unpack_from('<H12h', p)
        return 'imu', {'odr_hz': odr, 'samples': [s[0:6], s[6:12]]}
    if rec_id == 0x41:
        can_id, dlc = struct.unpack_from('<IB', p)
        return 'can_rx', {'can_id': '0x%X' % can_id, 'dlc': dlc, 'data': p[5:5 + min(dlc, 8)].hex()}
    return 'id_0x%02X' % rec_id, {'raw': p.hex()}


def read_file(path):
    data = Path(path).read_bytes()
    if len(data) < HEADER.size:
        raise ValueError('fichier trop court')
    (magic, version, stream, rec_size, session, boot_session, segment,
     imu_rate, fw, uid, tick_open) = HEADER.unpack_from(data)
    if magic != LOG_MAGIC:
        raise ValueError('magic invalide (pas un log du dashboard)')
    hdr = dict(version=version, stream=STREAMS.get(stream, stream), record_size=rec_size, session=session,
               boot_session=boot_session, segment=segment, imu_rate_hz=imu_rate, device_uid='%08X' % uid,
               tick_at_open=tick_open)
    rec = REC32 if rec_size == 32 else REC16
    events = []
    # un dernier record incomplet (coupure d'alimentation) est ignoré
    for off in range(HEADER.size, len(data) - rec.size + 1, rec.size):
        if rec is REC32:
            tick, rid, _flags, payload = rec.unpack_from(data, off)
        else:
            tick, rid, payload = rec.unpack_from(data, off)
        if rid == 0:           # remplissage
            continue
        name, fields = decode_payload(rid, payload)
        events.append({'t': tick / 1000.0, 'stream': hdr['stream'], 'event': name, **fields})
    return hdr, events


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='fichier(s) .bin ou dossier(s) de session')
    ap.add_argument('--stream', choices=['session', 'raw', 'sys'], help="ne garder qu'un flux")
    ap.add_argument('--csv', metavar='DOSSIER', help='écrit un CSV par type d\'événement')
    args = ap.parse_args()

    files = []
    for p in map(Path, args.paths):
        files += sorted(f for f in p.iterdir() if f.suffix.lower() == '.bin') if p.is_dir() else [p]   # FAT 8.3 : SES_00.BIN

    events = []
    for f in files:
        try:
            hdr, ev = read_file(f)
        except ValueError as e:
            print('# %s ignoré : %s' % (f.name, e), file=sys.stderr)
            continue
        print('# %s : flux=%s session=%d (boot %d) segment=%d imu=%d Hz uid=%s, %d événements'
              % (f.name, hdr['stream'], hdr['session'], hdr['boot_session'], hdr['segment'],
                 hdr['imu_rate_hz'], hdr['device_uid'], len(ev)), file=sys.stderr)
        events += ev

    if args.stream:
        events = [e for e in events if e['stream'] == args.stream]
    events.sort(key=lambda e: e['t'])        # fusion chronologique de tous les flux

    if args.csv:
        out = Path(args.csv)
        out.mkdir(parents=True, exist_ok=True)
        by_name = {}
        for e in events:
            by_name.setdefault(e['event'], []).append(e)
        for name, rows in by_name.items():
            cols = list(dict.fromkeys(k for r in rows for k in r))
            with open(out / (name + '.csv'), 'w', newline='', encoding='utf-8') as fh:
                w = csv.DictWriter(fh, fieldnames=cols)
                w.writeheader()
                w.writerows(rows)
        print('CSV écrits dans', out, file=sys.stderr)
    else:
        for e in events:
            fields = ' '.join('%s=%s' % (k, v) for k, v in e.items() if k not in ('t', 'stream', 'event'))
            print('%10.3f  %-8s %-10s %s' % (e['t'], e['stream'], e['event'], fields))


if __name__ == '__main__':
    main()
