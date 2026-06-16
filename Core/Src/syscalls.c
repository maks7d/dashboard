#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

// Déclaration de fonction pour éviter les warnings "implict declaration"
int _isatty(int file);
int _write(int file, char *ptr, int len);
int _close(int file);
int _lseek(int file, int ptr, int dir);
int _read(int file, char *ptr, int len);
int _fstat(int file, struct stat *st);

int _isatty(int file) {
    return 1;
}

int _write(int file, char *ptr, int len) {
    // Si on avait un UART configuré, on enverrait les caractères un par un.
    // Pour l'instant on fait juste semblant.
    for (int i = 0; i < len; i++) {
        // Envoi_UART(ptr[i]);  // À remplacer plus tard par votre code UART réel
    }
    return len;
}

int _close(int file) {
    return -1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}
