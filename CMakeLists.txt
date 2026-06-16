cmake_minimum_required(VERSION 3.15)
project(DashboardSTM32 C ASM)

# Configuration standard C
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Architecture STM32H7
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16")

# Flags de compilation C et Assembleur
set(CMAKE_C_FLAGS "${CPU_FLAGS} -O0 -g3 -Wall" CACHE INTERNAL "C Compiler options")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp -g3" CACHE INTERNAL "ASM Compiler options")

# Flags du Linker
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/Core/linker/stm32h745xx_flash_CM7.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -T \"${LINKER_SCRIPT}\" --specs=nano.specs --specs=nosys.specs -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref" CACHE INTERNAL "Linker options")

# Définitions du préprocesseur
add_definitions(-DSTM32H743xx)

# Dossiers d'en-tête (Includes)
include_directories(
    Core/include
    Core/Src
)

# Fichiers sources (on récupère automatiquement tous les .c et .s)
file(GLOB SOURCES 
    "Core/Src/*.c"
    "Core/Src/*.s"
)

# Créer l'exécutable principal (.elf)
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Commandes post-compilation pour générer les .bin / .hex et afficher la taille
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Génération de ${PROJECT_NAME}.bin et .hex et calcul de la taille"
)
