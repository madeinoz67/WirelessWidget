
---



---


# Drivers #
  * RFM12B Drivers by Jean-Claude of Jeelab http://jeelab.equi4.com/2009/02/10/rfm12b-library-for-arduino/.  His BLOG has a wealth of information regarding the RFM12B and low power usage.

# Bootloaders #
The original lilypad bootloader have been modified for the WidgetBoards. (the unmodified lilypad bootloader will also work with the widget board and no crystal)

|Clock Speed|Used when |Fuse Settings | D/L Link|
|:----------|:---------|:-------------|:--------|
|8MHz |Atmega168 No crystal installed|L:E2 H:DD E:00 |[rfm12widgetBOOT\_168\_8MHZ.hex](http://code.google.com/p/strobit/downloads/detail?name=rfm12widgetBOOT_168_8MHZ.hex)|
|10MHz|Atmega168 with crystal installed|L:F7 H:DD E:00|[rfm12widgetBOOT\_168\_10MHZ.hex](http://code.google.com/p/strobit/downloads/detail?name=rfm12widgetBOOT_168_10MHZ.hex)|
|8MHz |Atmega328p No crystal installed|L:E2 H:DA E:05 |[rfm12widgetBOOT\_168\_8MHZ.hex](http://code.google.com/p/strobit/downloads/detail?name=rfm12widgetBOOT_168_8MHZ.hex)|
|10MHz|Atmega328p with crystal installed|L:F7 H:DA E:05|[rfm12widgetBOOT\_168\_10MHZ.hex](http://code.google.com/p/strobit/downloads/detail?name=rfm12widgetBOOT_168_10MHZ.hex)|
|16MHz|Atmega328p with crystal installed|L:F7 H:DA E:05|[rfm12widgetBOOT\_168\_10MHZ.hex](http://code.google.com/p/strobit/downloads/detail?name=rfm12widgetBOOT_168_10MHZ.hex)|

# Netwoking #
> [Networking](Networking.md) the Widgets
    * [JeeLab](http://code.google.com/p/jeelab/) have a simple network that works well.
    * [WidgetMesh](WidgetMesh.md) - A Mesh network for the WidgetBoards

# Applications #


  * [WidgetMesh](WidgetMesh.md) - A Mesh network for the WidgetBoards
  * [JeeLab Ports](http://code.google.com/p/jeelab/wiki/Ports) wireless sensor application.
  * Others. See [Applications](Applications.md)