<?xml version="1.0" encoding="UTF-8"?>
<simconf>
  <project EXPORT="discard">[APPS_DIR]/mrm</project>
  <project EXPORT="discard">[APPS_DIR]/mspsim</project>
  <project EXPORT="discard">[APPS_DIR]/avrora</project>
  <project EXPORT="discard">[APPS_DIR]/serial_socket</project>
  <project EXPORT="discard">[APPS_DIR]/collect-view</project>
  <project EXPORT="discard">[APPS_DIR]/powertracker</project>
  <simulation>
    <title>My simulation</title>
    <randomseed>123456</randomseed>
    <motedelay_us>1000000</motedelay_us>
    <radiomedium>
      se.sics.cooja.radiomediums.UDGM
      <transmitting_range>70.0</transmitting_range>
      <interference_range>90.0</interference_range>
      <success_ratio_tx>1.0</success_ratio_tx>
      <success_ratio_rx>0.0</success_ratio_rx>
    </radiomedium>
    <events>
      <logoutput>40000</logoutput>
    </events>
    <motetype>
      se.sics.cooja.mspmote.SkyMoteType
      <identifier>sky1</identifier>
      <description>Sky Mote Type #sky1</description>
      <source EXPORT="discard">[CONTIKI_DIR]/examples/ipv6/rpl-collect/udp-sink.c</source>
      <commands EXPORT="discard">make udp-sink.sky TARGET=sky</commands>
      <firmware EXPORT="copy">[CONTIKI_DIR]/examples/ipv6/rpl-collect/udp-sink.sky</firmware>
      <moteinterface>se.sics.cooja.interfaces.Position</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.IPAddress</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.MoteAttributes</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspClock</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspMoteID</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyButton</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyFlash</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyCoffeeFilesystem</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.Msp802154Radio</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspSerial</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyLED</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspDebugOutput</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyTemperature</moteinterface>
    </motetype>
    <motetype>
      se.sics.cooja.mspmote.SkyMoteType
      <identifier>sky2</identifier>
      <description>Sky Mote Type #sky2</description>
      <source EXPORT="discard">[CONTIKI_DIR]/examples/ipv6/rpl-collect/udp-sender.c</source>
      <commands EXPORT="discard">make udp-sender.sky TARGET=sky</commands>
      <firmware EXPORT="copy">[CONTIKI_DIR]/examples/ipv6/rpl-collect/udp-sender.sky</firmware>
      <moteinterface>se.sics.cooja.interfaces.Position</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.IPAddress</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>se.sics.cooja.interfaces.MoteAttributes</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspClock</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspMoteID</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyButton</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyFlash</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyCoffeeFilesystem</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.Msp802154Radio</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspSerial</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyLED</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.MspDebugOutput</moteinterface>
      <moteinterface>se.sics.cooja.mspmote.interfaces.SkyTemperature</moteinterface>
    </motetype>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>242.83184008074136</x>
        <y>-88.93434685786869</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>1</id>
      </interface_config>
      <motetype_identifier>sky1</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>205.7645134037647</x>
        <y>-62.438740480554074</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>2</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>250.51864863077387</x>
        <y>-59.2420165357677</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>3</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>294.4736028715864</x>
        <y>-63.23792146675066</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>4</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>176.19481691449084</x>
        <y>-35.26658694986995</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>5</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>222.54731411389315</x>
        <y>-32.869043991280165</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>6</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>273.694897230475</x>
        <y>-29.672320046493798</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>7</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>321.64575640227054</x>
        <y>-33.66822497747676</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>8</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>159.4120162043624</x>
        <y>-2.500166515809672</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>9</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>196.97352255560222</x>
        <y>-0.10262355721989598</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>10</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>252.91619158936365</x>
        <y>1.495738415173288</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>11</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>301.66623174735577</x>
        <y>-0.10262355721989598</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>12</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>346.4203669743649</x>
        <y>1.495738415173288</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>13</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>124.24805281171236</x>
        <y>22.27444405628468</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>14</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>180.1907218454738</x>
        <y>35.86052082162674</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>15</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>224.14567608628633</x>
        <y>30.266253918250598</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>16</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>276.0924401890648</x>
        <y>35.86052082162674</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>17</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>351.2154528915445</x>
        <y>37.45888279401993</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>18</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>89.08408941906231</x>
        <y>47.04905462837903</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>19</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>180.1907218454738</x>
        <y>75.02038914525976</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>20</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>245.7235627135943</x>
        <y>66.22939829709723</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>21</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>290.4776979406035</x>
        <y>67.82776026949043</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>22</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>370.3957965602627</x>
        <y>64.63103632470406</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>23</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>93.07999435004527</x>
        <y>82.21301802102909</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>24</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
    <mote>
      <breakpoints />
      <interface_config>
        se.sics.cooja.interfaces.Position
        <x>204.16615143137156</x>
        <y>106.18844760692684</y>
        <z>0.0</z>
      </interface_config>
      <interface_config>
        se.sics.cooja.mspmote.interfaces.MspMoteID
        <id>25</id>
      </interface_config>
      <motetype_identifier>sky2</motetype_identifier>
    </mote>
  </simulation>
  <plugin>
    se.sics.cooja.plugins.SimControl
    <width>259</width>
    <z>5</z>
    <height>184</height>
    <location_x>0</location_x>
    <location_y>0</location_y>
  </plugin>
  <plugin>
    se.sics.cooja.plugins.Visualizer
    <plugin_config>
      <skin>se.sics.cooja.plugins.skins.IDVisualizerSkin</skin>
      <skin>se.sics.cooja.plugins.skins.AttributeVisualizerSkin</skin>
      <skin>se.sics.cooja.plugins.skins.UDGMVisualizerSkin</skin>
      <viewport>1.836243522352668 0.0 0.0 1.836243522352668 -124.43273668589376 177.8080782058222</viewport>
    </plugin_config>
    <width>610</width>
    <z>4</z>
    <height>482</height>
    <location_x>420</location_x>
    <location_y>-2</location_y>
  </plugin>
  <plugin>
    se.sics.cooja.plugins.LogListener
    <plugin_config>
      <filter />
      <formatted_time />
      <coloring />
    </plugin_config>
    <width>1347</width>
    <z>6</z>
    <height>150</height>
    <location_x>0</location_x>
    <location_y>438</location_y>
  </plugin>
  <plugin>
    se.sics.cooja.plugins.TimeLine
    <plugin_config>
      <mote>0</mote>
      <mote>1</mote>
      <mote>2</mote>
      <mote>3</mote>
      <mote>4</mote>
      <mote>5</mote>
      <mote>6</mote>
      <mote>7</mote>
      <mote>8</mote>
      <mote>9</mote>
      <mote>10</mote>
      <mote>11</mote>
      <mote>12</mote>
      <mote>13</mote>
      <mote>14</mote>
      <mote>15</mote>
      <mote>16</mote>
      <mote>17</mote>
      <mote>18</mote>
      <mote>19</mote>
      <mote>20</mote>
      <mote>21</mote>
      <mote>22</mote>
      <mote>23</mote>
      <mote>24</mote>
      <showRadioRXTX />
      <zoomfactor>500.0</zoomfactor>
    </plugin_config>
    <width>1347</width>
    <z>2</z>
    <height>150</height>
    <location_x>0</location_x>
    <location_y>588</location_y>
  </plugin>
  <plugin>
    se.sics.cooja.plugins.collectview.CollectView
    <mote_arg>0</mote_arg>
    <width>203</width>
    <z>3</z>
    <height>75</height>
    <location_x>120</location_x>
    <location_y>120</location_y>
  </plugin>
  <plugin>
    PowerTracker
    <width>400</width>
    <z>1</z>
    <height>400</height>
    <location_x>150</location_x>
    <location_y>150</location_y>
  </plugin>
  <plugin>
    se.sics.cooja.plugins.ScriptRunner
    <plugin_config>
      <script>/*&#xD;
 * Example Contiki test script (JavaScript).&#xD;
 * A Contiki test script acts on mote output, such as via printf()'s.&#xD;
 * The script may operate on the following variables:&#xD;
 *  Mote mote, int id, String msg&#xD;
 */&#xD;
&#xD;
/* Make test automatically fail (timeout) after 100 simulated seconds */&#xD;
//TIMEOUT(100000); /* milliseconds. no action at timeout */&#xD;
TIMEOUT(1800000); /* milliseconds. print last msg at timeout */&#xD;
&#xD;
while(1) {&#xD;
    log.log(id + ":" + time + ":" + msg + "\n");&#xD;
&#xD;
    YIELD(); /* wait for another mote output */&#xD;
}</script>
      <active>true</active>
    </plugin_config>
    <width>600</width>
    <z>0</z>
    <height>700</height>
    <location_x>677</location_x>
    <location_y>0</location_y>
  </plugin>
</simconf>

