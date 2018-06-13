# BzBoard_BETA
beta-release of development firmware for BuzzzBoard PCBA


## Use

See license file.


## Change Log

###v1
  - Initial release


## Local Build

    - Save to new folder.

    - Build using Particle CLI command 'particle compile photon . --saveTo firmware.bin'

    - Upload 'firmware.bin' to Photon using USB cable and DFU mode:

    - hold SETUP button on RESET until blinking YELLOW

    - Send 'particle flash --usb firmware.bin'

