const rootTime = 1616356678704 - 1

const endTime = [
  "2021-03-21T20:08:50.751Z",
  "2021-03-21T20:09:14.815Z",
  "2021-03-21T20:09:50.810Z",
  "2021-03-21T20:10:08.770Z",
  "2021-03-21T20:11:17.690Z",
  "2021-03-21T20:11:44.736Z",
  "2021-03-21T20:13:29.688Z",
  "2021-03-21T20:13:35.769Z",
  "2021-03-21T20:13:41.767Z",
]

const startTime = [
  651981,
  675981,
  711981,
  729981,
  798981,
  825981,
  930981,
  936981,
  942981,
].map(time => rootTime + time)

let transmitTime = []
for(let i = 0; i < endTime.length; i++) {
  transmitTime[i] = new Date(endTime[i]).getTime() - startTime[i]
}

console.log("傳輸時間", transmitTime)

let avgTransmitTime = 0
for(i = 0; i < transmitTime.length; i++) {
  avgTransmitTime += transmitTime[i]
}
avgTransmitTime = avgTransmitTime / transmitTime.length

console.log("avgTransmitTime milliseconds", avgTransmitTime)