#!/bin/bash
CURR_FOLDER=${PWD}
ROOT_FOLDER=${CURR_FOLDER}/bin
PLUGIN_FOLDER=${ROOT_FOLDER}/drake_plugins
VISUALIZER=${ROOT_FOLDER}/drake-visualizer
SHOW_FRAME=${PLUGIN_FOLDER}/show_frame.py
SHOW_TIME=${PLUGIN_FOLDER}/show_time.py
SHOW_HY_CONTACT=${PLUGIN_FOLDER}/show_hydroelastic_contact.py
SHOW_PP_CONTACT=${PLUGIN_FOLDER}/show_point_pair_contact.py
SHOW_IMAGE=${PLUGIN_FOLDER}/show_image.py

${VISUALIZER} --script ${SHOW_TIME} --script ${SHOW_FRAME}  --script ${SHOW_PP_CONTACT} --script ${SHOW_IMAGE} --script ${SHOW_HY_CONTACT}
