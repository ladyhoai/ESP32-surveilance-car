const express = require('express')
const client_port = express()
.use((req, res) => {
    const ip = req.socket.remoteAddress;
    console.log(`New client connected with local IP ${ip}`);
    if (ip != '192.168.1.36') {
        res.sendFile('./index.html', {root: __dirname});
    }
})
.use(express.static(__dirname))
.listen(4000,() => console.log(`Server is listening on port ${4000}`))


const seph_port = express()
.use((req, res) => console.log("Seph is connected and ready for operation"))
.listen(4001)

const { WebSocket } = require('ws');

const wss = new WebSocket.Server({server : client_port });
const seph_command = new WebSocket.Server({server : seph_port});

wss.binaryType = 'arraybuffer'


wss.on('connection', function connection(ws, req) {
    const client_ip = req.socket.remoteAddress;

    ws.on('message', function incoming(data) {
        console.log(data)
        wss.clients.forEach(function each(client) {
            if (client !== ws && client.readyState === WebSocket.OPEN) {
                client.send(data.toString());
            }
        })
    })

    ws.on('close', () => {console.log(`Client with IP ${client_ip} disconnected`)})
})

seph_command.on('connection', (seph, req) => {
    const feature = req.headers['identifier'];
    console.log(`Seph: ${feature} connected`);
    let audio_buffer = Buffer.from([])
    
    seph.on('message', (data) => {          
        seph_command.clients.forEach(client => {
            if (client !== seph && client.readyState === WebSocket.OPEN) {
                client.send(data)//.buffer);
            }
        })            
    })

    seph.on('close', () => {
        console.log("seph disconnected");
    })
})
