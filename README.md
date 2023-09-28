# kproxmark3_sim

A test project to handle SIM module of the Proxmark3, in C
Includes legacy `sim011.asm` and `sim013.asm` building

## Dependencies & Tools

- `sim011.asm` and `sim013.asm`, from Sentinel - original Proxmark3 - https://github.com/RfidResearchGroup/proxmark3/tree/master/tools/simmodule
- `ldrom.bin`, from @wh201906 - https://github.com/wh201906/PM3-Modding/tree/main/PM3_ISO7816
- `hex2bin`, from Jacques Pelletier - https://hex2bin.sourceforge.net/ & https://sourceforge.net/projects/hex2bin/


- Keil uVision v5,  the C51 development tools,  https://www.keil.com/download/

## How to compile
  `F7` to compile the project and in the project folder there is a sub folder called "Objects".
  The artifact you want is `sim_c.bin`

## How to install fw on Proxmark3 RDV4 sim module?

  - Take the `sim_c.bin` and copy to the proxmark3\client\resource folder.
  
  - Generate a sha512 file need when running the upgrade process inside pm3 client.
  
  `sha512sum -b client/resources/sim_c.bin > client/resources/sim_c.sha512.txt`

  - Start the Proxmark3 client and run
    `smart upgrade -f sim_c.bin`

  if all goes well,  you will have the following output
  
  ![Output of a successful run of the command Smart upgrade](/smart_upgrade_success.png)

  