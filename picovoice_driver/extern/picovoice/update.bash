#!/usr/bin/env bash

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Headers
wget -nv https://raw.githubusercontent.com/Picovoice/rhino/master/include/picovoice.h -O $SCRIPT_DIR/include/picovoice.h
wget -nv https://raw.githubusercontent.com/Picovoice/porcupine/master/include/pv_porcupine.h -O $SCRIPT_DIR/include/pv_porcupine.h
wget -nv https://raw.githubusercontent.com/Picovoice/rhino/master/include/pv_rhino.h -O $SCRIPT_DIR/include/pv_rhino.h

# Libs x86_64
wget -nv https://github.com/Picovoice/porcupine/raw/master/lib/linux/x86_64/libpv_porcupine.so -O $SCRIPT_DIR/lib/x86_64/libpv_porcupine.so
wget -nv https://github.com/Picovoice/rhino/raw/master/lib/linux/x86_64/libpv_rhino.so -O $SCRIPT_DIR/lib/x86_64/libpv_rhino.so

# Resource contexts
wget -nv https://github.com/Picovoice/rhino/raw/master/resources/contexts/linux/alarm_linux.rhn -O $SCRIPT_DIR/resources/contexts/alarm_linux.rhn
wget -nv https://github.com/Picovoice/rhino/raw/master/resources/contexts/linux/clock_linux.rhn -O $SCRIPT_DIR/resources/contexts/clock_linux.rhn
wget -nv https://github.com/Picovoice/rhino/raw/master/resources/contexts/linux/coffee_maker_linux.rhn -O $SCRIPT_DIR/resources/contexts/coffee_maker_linux.rhn
wget -nv https://github.com/Picovoice/rhino/raw/master/resources/contexts/linux/smart_lighting_linux.rhn -O $SCRIPT_DIR/resources/contexts/smart_lighting_linux.rhn

# Resource keywords
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/alexa_linux.ppn -O $SCRIPT_DIR/resources/keywords/alexa_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/americano_linux.ppn -O $SCRIPT_DIR/resources/keywords/americano_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/blueberry_linux.ppn -O $SCRIPT_DIR/resources/keywords/blueberry_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/bumblebee_linux.ppn -O $SCRIPT_DIR/resources/keywords/bumblebee_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/computer_linux.ppn -O $SCRIPT_DIR/resources/keywords/computer_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/grapefruit_linux.ppn -O $SCRIPT_DIR/resources/keywords/grapefruit_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/grasshopper_linux.ppn -O $SCRIPT_DIR/resources/keywords/grasshopper_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/hey%20google_linux.ppn -O $SCRIPT_DIR/resources/keywords/hey_google_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/hey%20siri_linux.ppn -O $SCRIPT_DIR/resources/keywords/hey_siri_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/jarvis_linux.ppn -O $SCRIPT_DIR/resources/keywords/jarvis_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/ok%20google_linux.ppn -O $SCRIPT_DIR/resources/keywords/ok_google_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/pico%20clock_linux.ppn -O $SCRIPT_DIR/resources/keywords/pico_clock_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/picovoice_linux.ppn -O $SCRIPT_DIR/resources/keywords/picovoice_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/porcupine_linux.ppn -O $SCRIPT_DIR/resources/keywords/porcupine_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/smart%20mirror_linux.ppn -O $SCRIPT_DIR/resources/keywords/smart_mirror_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/snowboy_linux.ppn -O $SCRIPT_DIR/resources/keywords/snowboy_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/terminator_linux.ppn -O $SCRIPT_DIR/resources/keywords/terminator_linux.ppn
wget -nv https://github.com/Picovoice/porcupine/raw/master/resources/keyword_files/linux/view%20glass_linux.ppn -O $SCRIPT_DIR/resources/keywords/view_glass_linux.ppn

# Resource models
wget -nv https://github.com/Picovoice/porcupine/raw/master/lib/common/porcupine_params.pv -O $SCRIPT_DIR/resources/models/porcupine_params.pv
wget -nv https://github.com/Picovoice/rhino/raw/master/lib/common/rhino_params.pv -O $SCRIPT_DIR/resources/models/rhino_params.pv
