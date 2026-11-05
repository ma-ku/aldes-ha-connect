# Aldes HA-Connect

Replacing the Aldes ConnectBox USB with an ESPHome component.

## Credits

This project is mainly a rebuild of the work in 

  https://github.com/kikisarem/aldes-pilot-community

All credits go to them!

This project is published using the same license as the original project, keep
it free.

# Origin

It originated from the fact that flashing a Raspberry Pico W every time to
change the firmware along with the corresponding HomeAssistant plugin appeared
to complex. The idea was to accomplish the same with ESPHome. Luckily, the S3
series provide basically the same functionality as the Pico W and with the help
of AI the majority of the code could be re-implemented on the ESPHome platform.

Since the project relies on custom components, the installation requires access
to this repository to download these two components.
