import requests

#Request class to send post requests to ESP32
class Request():
    def __init__(self):
        self.forwardurl = "http://192.168.4.1/forward"
        self.backwardsurl = "http://192.168.4.1/backward"
        self.righturl = "http://192.168.4.1/right"
        self.lefturl = "http://192.168.4.1/left"
        self.stopurl = "http://192.168.4.1/stop"
        self.autourl = "http://192.168.4.1/auto"
        self.reqObj = {'action': 'true'}

    #Send post request to endpoint for button/direction
    def sendRequest(self, action: str):
        if action == "forward":
            url = self.forwardurl
        elif action == "backwards":
            url = self.backwardsurl
        elif action == "right":
            url = self.righturl
        elif action == "left":
            url = self.lefturl
        elif action == "stop":
            url = self.stopurl
        elif action == "auto":
            url = self.autourl
        
        requests.post(url, json = self.reqObj)