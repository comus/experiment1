const fs = require('fs-extra')

const logFile = `./logs/${Date.now()}.csv`
fs.ensureFileSync(logFile)
const log = fs.createWriteStream(logFile, { flags: 'a' });

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('/dev/cu.SLAB_USBtoUART3', {
  baudRate: 115200
})

const parser = port.pipe(new Readline())
parser.on('data', data => {
  // console.log('data', data)

  if (data.includes('[!]')) {
    const arr = data.split(',')
    arr.shift()
    arr.pop()
    console.log(arr.join(','))
    log.write(arr.join(',') + '\n')
  }
})


var readline = require('readline')
var rl = readline.createInterface(process.stdin, process.stdout);

rl.prompt();

rl.on('line', function(line) {
  const cmd = line.trim()
    if (cmd === 'hello') {
      console.log('world!');
    }
    rl.prompt();
}).on('close', function() {
    console.log('Have a great day!');
    log.end();
    process.exit(0);
});
