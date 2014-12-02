import OSC
c = OSC.OSCClient()
c.connect(('localhost', 7110))   # connect to SuperCollider
oscmsg = OSC.OSCMessage()
oscmsg.setAddress("/startup")
oscmsg.append('HELLO')
c.send(oscmsg)
