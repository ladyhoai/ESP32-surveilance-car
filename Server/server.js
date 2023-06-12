const express = require('express')
const client_port = express()
.use((req, res) => {
    const ip = req.socket.remoteAddress;
    console.log(`New client connected with local IP ${ip}`);
        res.sendFile('./index.html', {root: __dirname});
    
})
.use(express.static(__dirname))
.listen(4000,() => console.log(`Server is listening on port ${4000}`))

const seph_port = express()
.use((req, res) => console.log("Seph is connected and ready for operation"))
.listen(4001)

const camera_port = express()
.use((req, res) => console.log("camera connected"))
.listen(4002)

const { WebSocket } = require('ws');

var camera_ip = ""
var connection_ID = {}
const wss = new WebSocket.Server({server : client_port });
const seph_command = new WebSocket.Server({server : seph_port});
const camera = new WebSocket.Server({server : camera_port});

wss.binaryType = 'arraybuffer'
seph_command.binaryType = 'arraybuffer'
wss.on('connection', function connection(ws, req) {
    const client_ip = req.socket.remoteAddress;
    console.log(`New client connected with local IP  ${client_ip}`)
    
    ws.on('message', function incoming(data) {
        // save the IP address of esp32 camera
        //console.log(Buffer.from(new Uint8Array(data)).toString())
        if (data.byteLength == 3) {
            var temp_identifier = Buffer.from(new Uint8Array(data)).toString()
            if (temp_identifier == "cam") {
                connection_ID[ws] = "camera"; 
                console.log("Camera connected")
            }

            else if (temp_identifier == "esp") {
                connection_ID[ws] = "esp32";
                console.log("Car connected")
            }
        }

        
        wss.clients.forEach(function each(client) {
            if (client !== ws && client.readyState === WebSocket.OPEN) {     
                client.send(data)
            }
        })
    })

    ws.on('close', () => {
        console.log(`Client with IP ${client_ip} disconnected`)
        connection_ID[ws] = ""
    })

    ws.onerror = (error) => {
        console.log(error.message)
    } 
})

seph_command.on('connection', (seph, req) => {
    console.log(`Seph connected`);

    seph.on('message', (data) => {       
        seph_command.clients.forEach(client => {
            if (client !== seph && client.readyState === WebSocket.OPEN) {
                //client.send(buf)
                client.send(data) // Send audio data, previous is: data.buffer 
            }
        })            
    })

    seph.on('close', () => {
        console.log("seph disconnected");
    })
    seph.onerror = (error) => {
        console.log(error.message)
    } 
})

camera.on('connection', (cam_client, req) => {

    console.log("camera connected")
    cam_client.on('message', (data) => {       

        camera.clients.forEach(function each(client) {
            if (client !== cam_client && client.readyState === WebSocket.OPEN) {     
                client.send(data)
            }
        })   
    })

    cam_client.on('close', () => {
        console.log("camera client disconnected");
    })

    cam_client.onerror = (error) => {
        console.log(error.message)
    } 
})
