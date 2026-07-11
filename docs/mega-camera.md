# Mega Camera and SD

The Mega camera controller uses the official `Arducam_Mega` library for the
Arducam Mega B0401. Install that library from the Arduino IDE Library Manager
before compiling the Mega sketch. Do not substitute the older `ArduCAM` library.

The camera and SD card share the Mega hardware SPI bus on the ICSP header. The
camera chip-select is D7 and the SD chip-select is D4. Both are driven HIGH
whenever the other device is accessed.

Valid capture commands are processed as a nonblocking burst. The B0401 creates
QVGA JPEGs, each image is written as `RV00001.JPG` through `RV99999.JPG`, and
existing files are never overwritten. The default manual-photo command creates
10 images at 500 ms intervals. Camera, SD, malformed-JPEG, and receive-timeout
failures abort the active burst and are reported on the Mega USB serial console.

An SD-eject command waits only for the current image to finish, and cancels
any remaining images in the burst. It then closes the file, calls `SD.end()`,
deselects both SPI devices, and reports
`SD card is safe to remove.` on the Mega USB serial console. Further capture
requests are rejected until the Mega is restarted with the card installed.
