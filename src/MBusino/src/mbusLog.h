// M-Bus debug log for the web interface (/mbus): the last received telegrams and the last decoded telegram.
// Only MBusinoNano, the other variants get empty functions.
// WARNING: the web server runs in a separate FreeRTOS task, every access is guarded by mbusLogMutex!

#if defined(MBUSINO_ETH)

#define MBUS_LOG_SIZE 10 // number of telegrams kept in the log

struct mbusLogEntry {
  uint32_t time;    // millis() when the entry was added
  uint8_t status;   // MBUS_LOG_...
  uint8_t address;  // requested M-Bus address
  uint16_t len;     // number of valid bytes in data
  uint8_t data[MBUS_DATA_SIZE];
};

mbusLogEntry mbusLog[MBUS_LOG_SIZE];
uint8_t mbusLogNext = 0;   // ring buffer position of the next entry
uint32_t mbusLogCount = 0; // number of entries since start
char mbusLastJson[sizeof(jsonstring)] = {0}; // last decoded telegram (header + records)
uint8_t mbusLastAddress = 0;
uint32_t mbusLastTime = 0;
SemaphoreHandle_t mbusLogMutex = NULL;

void mbusLogBegin(){
  mbusLogMutex = xSemaphoreCreateMutex();
}

// length of a received frame, a bad frame can be incomplete
uint16_t mbusFrameLength(const uint8_t *data){
  if(data[0] == 0xE5){
    return 1;
  }
  if(data[0] == 0x68){
    return min((uint16_t)(data[1] + 6), (uint16_t)MBUS_DATA_SIZE);
  }
  return 0;
}

// add a poll result to the log, data can be NULL if nothing was received
void mbusLogAdd(uint8_t status, uint8_t address, const uint8_t *data){
  if(mbusLogMutex == NULL || xSemaphoreTake(mbusLogMutex, pdMS_TO_TICKS(100)) != pdTRUE){
    return;
  }
  mbusLogEntry &entry = mbusLog[mbusLogNext];
  entry.time = millis();
  entry.status = status;
  entry.address = address;
  entry.len = (data != NULL) ? mbusFrameLength(data) : 0;
  if(entry.len > 0){
    memcpy(entry.data, data, entry.len);
  }
  mbusLogNext = (mbusLogNext + 1) % MBUS_LOG_SIZE;
  mbusLogCount++;
  xSemaphoreGive(mbusLogMutex);
}

// store the decoded telegram, json is the serialized header + records
void mbusLogSetJson(uint8_t address, const char *json){
  if(strlen(json) >= sizeof(mbusLastJson) - 1){ // probably truncated, no valid JSON
    return;
  }
  if(mbusLogMutex == NULL || xSemaphoreTake(mbusLogMutex, pdMS_TO_TICKS(100)) != pdTRUE){
    return;
  }
  strcpy(mbusLastJson, json);
  mbusLastAddress = address;
  mbusLastTime = millis();
  xSemaphoreGive(mbusLogMutex);
}

// GET /mbus.json, the log newest first and the last decoded telegram
void handleMbusJson(AsyncWebServerRequest *request){
  String out;
  out.reserve(MBUS_LOG_SIZE * (MBUS_DATA_SIZE * 2 + 48) + sizeof(mbusLastJson) + 96);
  if(mbusLogMutex == NULL || xSemaphoreTake(mbusLogMutex, pdMS_TO_TICKS(500)) != pdTRUE){
    request->send(503, "text/plain", "busy");
    return;
  }
  out += "{\"now\":";
  out += millis();
  out += ",\"count\":";
  out += mbusLogCount;
  out += ",\"next\":"; // seconds until the next poll, -1 while a poll is running
  if(mbusLoopStatus == 0){
    unsigned long elapsed = millis() - timerMbus;
    out += (elapsed < userData.mbusInterval) ? (userData.mbusInterval - elapsed) / 1000 : 0;
  }else{
    out += -1;
  }
  out += ",\"log\":[";
  uint8_t entries = min(mbusLogCount, (uint32_t)MBUS_LOG_SIZE);
  for(uint8_t i = 0; i < entries; i++){
    const mbusLogEntry &entry = mbusLog[(mbusLogNext + MBUS_LOG_SIZE - 1 - i) % MBUS_LOG_SIZE];
    if(i > 0){
      out += ',';
    }
    out += "{\"t\":";
    out += entry.time;
    out += ",\"s\":";
    out += entry.status;
    out += ",\"a\":";
    out += entry.address;
    out += ",\"hex\":\"";
    char hex[3];
    for(uint16_t b = 0; b < entry.len; b++){
      snprintf(hex, sizeof(hex), "%02X", entry.data[b]);
      out += hex;
    }
    out += "\"}";
  }
  out += "],\"last\":";
  if(mbusLastJson[0] != 0){
    out += "{\"t\":";
    out += mbusLastTime;
    out += ",\"a\":";
    out += mbusLastAddress;
    out += ",\"data\":";
    out += mbusLastJson;
    out += '}';
  }else{
    out += "null";
  }
  out += '}';
  xSemaphoreGive(mbusLogMutex);
  request->send(200, "application/json", out);
}

// POST /mbus/poll, start the next M-Bus poll right away
void handleMbusPoll(AsyncWebServerRequest *request){
  if(mbusLoopStatus == 0){
    timerMbus = millis() - userData.mbusInterval - 1; // the interval is over, the loop polls at its next run
  }
  request->send(200, "text/plain", "ok");
}

const char mbus_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang='en'>
  <head>
    <meta charset='utf-8'>
    <meta name='viewport' content='width=device-width,initial-scale=1'>
    <title>MBusino M-Bus</title>
    <style>
      body {
        margin: 0;
        padding: 15px;
        font-family: 'Segoe UI', Roboto, 'Helvetica Neue', Arial, 'Noto Sans', 'Liberation Sans';
        font-size: 1rem;
        line-height: 1.5;
        color: #fff;
        background-color: #438287
      }

      main {
        max-width: 960px;
        margin: auto
      }

      h1 {
        text-align: center
      }

      h2 {
        font-size: 1.2rem;
        margin: 1.5rem 0 .5rem
      }

      table {
        border-collapse: collapse;
        width: 100%
      }

      th,
      td {
        text-align: left;
        vertical-align: top;
        padding: .2rem .5rem;
        border-bottom: 1px solid #6aa5aa
      }

      .wrap {
        overflow-x: auto
      }

      .hex {
        font-family: monospace;
        word-break: break-all
      }

      .ok {
        color: #c8f7c8
      }

      .err {
        color: #ffc8c8
      }

      button {
        cursor: pointer;
        color: #fff;
        background-color: #304648;
        border: 1px solid #304648;
        padding: .3rem 1rem;
        font-size: 1rem;
        border-radius: .3rem
      }
    </style>
  </head>
  <body>
    <main>
      <h1><i>MBusino</i> M-Bus</h1>
      <p><span id='info'>loading...</span> <button id='poll' onclick='poll()'>Poll now</button></p>
      <h2>Last decoded telegram</h2>
      <div class='wrap'><table id='header'></table></div><br>
      <div class='wrap'><table id='records'></table></div>
      <h2>Received telegrams</h2>
      <div class='wrap'><table id='log'></table></div>
      <p style='text-align:right'><a href='/' style='color:#3F4CFB'>home</a></p>
    </main>
    <script>
      const STATUS = ['OK', 'no data received', 'bad frame'];
      function esc(v) { return String(v ?? '').replace(/[&<>"']/g, c => '&#' + c.charCodeAt(0) + ';') }
      function age(ms) { const s = Math.round(ms / 1000); return s < 120 ? s + ' s ago' : Math.round(s / 60) + ' min ago' }
      function row(cells, th) { const t = th ? 'th' : 'td'; return '<tr>' + cells.map(c => '<' + t + '>' + c + '</' + t + '>').join('') + '</tr>' }
      function render(d) {
        document.getElementById('info').textContent = 'Polls since start: ' + d.count + ', ' + (d.next < 0 ? 'polling...' : 'next poll in ' + d.next + ' s');
        let h = row(['no telegram decoded yet']), r = '';
        if (d.last) {
          const hd = d.last.data.header || {};
          h = row(['Header, address ' + d.last.a + ', ' + age(d.now - d.last.t), ''], 1);
          for (const k in hd) {
            if (hd[k] && typeof hd[k] === 'object') { for (const s in hd[k]) h += row([esc(s), esc(hd[k][s])]) }
            else h += row([esc(k), esc(hd[k])]);
          }
          r = row(['#', 'name', 'value', 'unit', 'storage', 'tariff', 'subunit'], 1);
          (d.last.data.records || []).forEach((x, i) => {
            r += row([i + 1, esc(x.name), esc(x.value_string || x.value_scaled), esc(x.units), esc(x.storage), esc(x.tariff), esc(x.subUnit)]);
          });
        }
        document.getElementById('header').innerHTML = h;
        document.getElementById('records').innerHTML = r;
        let l = row(['received', 'address', 'status', 'bytes', 'telegram (hex)'], 1);
        d.log.forEach(e => {
          l += row([age(d.now - e.t), e.a, '<span class=' + (e.s ? 'err' : 'ok') + '>' + STATUS[e.s] + '</span>', e.hex.length / 2,
            '<span class=hex>' + e.hex.replace(/(..)(?!$)/g, '$1 ') + '</span>']);
        });
        document.getElementById('log').innerHTML = l;
      }
      async function poll() {
        document.getElementById('poll').disabled = true;
        try { await fetch('/mbus/poll', { method: 'POST' }) } catch (e) {}
        setTimeout(() => document.getElementById('poll').disabled = false, 3000);
      }
      async function update() {
        try { const res = await fetch('/mbus.json'); render(await res.json()) }
        catch (e) { document.getElementById('info').textContent = 'no connection to MBusino' }
        setTimeout(update, 2000);
      }
      update();
    </script>
  </body>
</html>)rawliteral";

void mbusLogSetupServer(){
  server.on("/mbus.json", HTTP_GET, handleMbusJson);
  server.on("/mbus/poll", HTTP_POST, handleMbusPoll); // before "/mbus", that handler matches "/mbus/..." too
  server.on("/mbus", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", mbus_html);
  });
}

#else
void mbusLogBegin(){}
void mbusLogAdd(uint8_t status, uint8_t address, const uint8_t *data){}
void mbusLogSetJson(uint8_t address, const char *json){}
void mbusLogSetupServer(){}
#endif
