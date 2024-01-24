import network
import usocket as socket
import ujson as json
from machine import reset
import binascii
class WiFiConfigurator:
    def __init__(self, ap_ssid="ESP32-Lathe", ap_password="password"):
        self.ap_ssid = ap_ssid
        self.ap_password = ap_password
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(essid=ap_ssid, password=ap_password)
        self.sta = network.WLAN(network.STA_IF)
        #self.mac = self.sta.config('mac')

    def unquote(self, string):
        bits = string.split('%')

        res = bits[0]
        for item in bits[1:]:
            res = res + chr(int(item[:2], 16)) + item[2:]

        return res
    
    def scan_wifi_networks(self):
        self.sta.active(True)
        networks = self.sta.scan()
        ssid_list = [network[0].decode("utf-8") for network in networks]
        self.sta.active(False)
        return ssid_list

    def start_ap_web_interface(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('192.168.4.1', 80))
        s.listen(1)

        print(f"Access Point started. Connect to '{self.ap_ssid}' with password '{self.ap_password}'.")
        print("Open a web browser and go to http://192.168.4.1/")

        while True:
            conn, addr = s.accept()
            print('Got a connection from %s' % str(addr))
            request = conn.recv(1024)
            request = str(request)
            print('Content = %s' % request)

            # Scan available WiFi networks
            ssid_list = self.scan_wifi_networks()

            # Parse the received request for SSID, password, and setup option
            ssid_start = request.find('ssid=')
            ssid_end = request.find('&', ssid_start)
            password_start = request.find('password=')
            password_end = request.find('&', password_start)
            remote_start = request.find('&remotes=')
            remote_end = request.find("\'", remote_start)
            
            
            #setup_option_end = request.find('HTTP', password_end)
            print("password start end")
            print(ssid_start, ssid_end, password_start, password_end, remote_start, remote_end)
            if ssid_start != -1 and password_start != -1:
                ssid = request[ssid_start + 5:ssid_end]
                password = request[password_start + 9:password_end]
                remotes = request[remote_start + 9:remote_end]
                print(ssid, password, remotes)
                # Configure the ESP32 with the provided SSID and password
                #self.ap.config(essid=ssid, password=password)

                # Save the SSID and password in a JSON file with first_run set to True
                self.save_config_to_json(ssid, password, remotes, first_run=True)

            # Send the web page with the list of available WiFi networks
            self.web_page(conn, ssid_list)
            
            # Close the connection after sending the response
            conn.close()

    def save_config_to_json(self, ssid, password, remotes, first_run=True):
        config = []

        remote = self.unquote(remotes)
        remote_list = remote.split(",")

        with open("settings.json") as f:
            config = json.load(f)

        config["firstrun"]["complete"] = True
        config["network"]["ssid"] = self.unquote(ssid)
        config["network"]["dwssap"] = self.unquote(password)
        config["mac"]["address"] = binascii.hexlify(self.sta.config('mac'))
        config["remote"] = remote_list
        with open("settings.json", "w") as config_file:
            json.dump(config, config_file)
        reset()

    def web_page(self, conn, ssid_list):
        dropdown_options = ""
        for ssid in ssid_list:
            dropdown_options += '<option value="{}">{}</option>'.format(ssid, ssid)

        html = """
        <html>
            <head>
                <title>Lathe Control WiFi Configuration</title>
            </head>
            <body>
                <h2>Lathe WiFi Configuration</h2>
                <h3>This is ONLY required to update firmware. Check the appropriate box to ignore wifi setup</h3>
                <form action="/" method="post">
                    <label for="ssid">Select SSID:</label>
                    <select id="ssid" name="ssid" required>
                        {}
                    </select><br><br>
                    <label for="password">Password:</label>
                    <input type="password" id="password" name="password" required><br><br>
                    <label for="setup">Ignore WiFi Setup:</label>
                    <input type="checkbox" id="setup" name="setup" checked><br><br>
                    <label for="remotes">Remote Setup - paste comma separated mac address of remote controls here</label>
                    <input type="text" id="remotes" name="remotes"><br><br>
                    <input type="submit" value="Submit">
                </form>
            </body>
        </html>
        """.format(dropdown_options)

        conn.send("HTTP/1.1 200 OK\n")
        conn.send("Content-Type: text/html\n")
        conn.send("Connection: close\n\n")
        conn.sendall(html)

# Create an instance of the WiFiConfigurator class
#wifi_configurator = WiFiConfigurator()

# Uncomment the following line to start the access point and web interface
# wifi_configurator.start_ap_web_interface()
