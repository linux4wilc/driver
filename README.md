# Microchip Embedded Wi-Fi®

<a href="http://www.microchip.com/design-centers/wireless-connectivity/embedded-wi-fi"><img src="http://www.microchip.com/_images/logo.png" align="left" hspace="10" vspace="6"></a>
</br></br></br>

Microcip Embedded Wi-Fi® is a family of self-contained, low power and certified modules bringing wireless internet connectivity.

**[ATWILC1000](http://www.microchip.com/wwwproducts/en/ATWILC1000)** is IEEE 802.11 b/g/n/ IOT link controller utilizing highly optimized 802.11 and provides mulitple peripheral interfaces like SPI, and SDIO.  

**[ATWILC3000](http://www.microchip.com/wwwproducts/en/ATWILC3000)** is IEEE 802.11 b/g/n/BT4.0 IOT link controller utilizing highly optimized 802.11-Bluetooth coexistence protocol and provides mulitple peripheral interfaces like UART, SPI, and SDIO.

This is the unified driver source for wilc1000 & wilc3000 chipsets.

Refer to the [Wi-Fi Link Controller Linux User Guide](http://ww1.microchip.com/downloads/en/DeviceDoc/ATWILC1000-ATWILC3000-Wi-Fi-Link-Controller-Linux-User-Guide-DS70005328B.pdf) for information on how to use the wireless devices on linux and the [Wi-Fi Link Controller Linux Release Notes](http://ww1.microchip.com/downloads/en/DeviceDoc/WSGA-3295A.pdf) for the latest release notes and revision history.

For more information on Microchip Embedded Wi-Fi®, visit [Microchip Embedded Wi-Fi®](http://www.microchip.com/design-centers/wireless-connectivity/embedded-wi-fi).

<h1>ATWILC Features</h1>

The ATWILC module supports the following features.
<ol type="1">
 <li><b>Wi-Fi Station (STA)</b>
  <ul>
   <li>IEEE 802.11 b/g/n</li>
   <li>Open, Wired Equivalent Privacy (WEP), Wi-Fi Protected Access (WPA)/WPA2 personal and
WPA/WPA2 enterprise security</li>
  </ul>
 </li>
 <li><b>Wi-Fi Access Point (AP)</b>
  <ul>
   <li>IEEE 802.11 b/g/n</li>
   <li>Open, WEP, WPA/WPA2 personal and WPA/WPA2 enterprise security</li>
   <li>Supports eight stations</li>
  </ul></li>
 <li><b>Wi-Fi Protected Setup (WPS)</b>
  <ul>
   <li>PBC</li>
   <li>PIN code</li>
  </ul>
 </li>
  <li><b>Wi-Fi direct</b>
   <ul>
    <li>P2P Client</li>
    <li>P2P GO</li>
   </ul>
  </li>
 <li><b>Concurrent modes</b>
  <ul>
   <li>STA-STA</li>
   <li>STA-AP</li>
   <li>STA-P2 Client</li>
   <li>STA-P2P GO</li>
   <li>AP-P2P Client</li>
  </ul>
 </li>
 <li><b>Antenna diversity control for Wi-Fi</b>
 </li>
 <li><b>Bluetooth (ATWILC3000 only)</b>
  <ul>
   <li>Bluetooth Low Energy (BLE) 4.0 support</li>
   <li>Modes of operation: Central and peripheral support</li>
   <li>Number of Connections: Supports seven clients</li>
   <li>Adaptive frequency hopping</li>
   <li>Coexistence with Wi-Fi</li>
  </ul>
 </li>
 <li><b>Power save</b>
  <ul>
   <li>Beacon monitoring mode</li>
   <li>Low-power mode when disconnected</li>
   <li>Host suspend support</li>
   <li>Wake-up host on wireless LAN events</li>
  </ul>
 </li>
 <li><b>RF version number 01.1</b>
 </li>
Note: RF version number format is xx.y, where xx: "Major" and y: "Minor".
Changes in Major number requires re-tests and possibly re-certification.
</ol>
