# LEGO_Printer

This project relative to Sensetime SDK.

You should download SDK first , and move the following file to the correct place.

## step1 - Build SDK

Read the README.md of SDK, to Build the SDK first.

## step2 - Move file

Cover the sample_common_align_3d.cpp with the same name file in /SDK/sdk_face-6.1.0-default0-linux-c0d8b71/samples/face

Move [myfile.txt]   [PID.py]   [legoCoreC.py]  into the build file.

## step3 - Build again

Run the make again.

## step4 - Run camera catch

cd to the build folder

    (1)./test_sample_common_align_3d
    (2) when you see your face in the screen ,press [space key] to catch the photo.
    (3) press the [ESC key] to close the process.

## step5 - Run printer

cd to the build folder

```
python3 legoCoreC.py
```

Wait until print complete.


