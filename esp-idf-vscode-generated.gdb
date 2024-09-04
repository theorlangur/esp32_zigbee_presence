target extended-remote :3333
set remotetimeout 20
symbol-file /home/orlangur/myapps/cpp/esp/projects/zb_co2/build/zb_co2.elf
mon reset halt
maintenance flush register-cache
thb app_main