// Delta step generator for the Orin Nano chess controller.
//
// The Orin plans every move as a stream of short segments, each a number of
// steps per axis spread evenly over a fixed time. This sketch queues them and
// runs them back to back, so the three cranks stay in step and the platform
// follows the straight line the Orin planned. All the timing lives here,
// because Linux on the Orin cannot hold a pulse train steady.
//
// One command per line, each answered "ok" or "err <reason>" once it is done:
//   S a b c us   queue a segment: a, b, c steps on each axis over us microseconds
//   W            wait until every queued segment has run
//   H            home: raise each arm onto its limit switch, which is step 0
//   E 0|1        electromagnet off or on, once the arms have stopped
//   ?            positions, switches, turn button, homed
//
// Positive steps move an arm down, away from its switch, so a valid position
// is never negative. The pins match an Arduino CNC Shield V3.

const byte STEP[3]  = {2, 3, 4};
const byte DIR[3]   = {5, 6, 7};
const byte LIMIT[3] = {9, 10, 11};   // Switch to GND, closed at the top of travel
const byte ENABLE   = 8;             // Shared driver enable, active low
const byte MAGNET   = 12;            // MOSFET or relay board for the electromagnet
const byte BUTTON   = A2;            // "My move is done", to GND
const bool DOWN_HIGH[3] = {true, true, true};   // DIR level that moves each arm down, set at bring-up

const long HOME_SPEED   = 400;       // Final approach to the switch [steps/s]
const long HOME_BACKOFF = 130;       // Back off this far before re-homing, about 3 deg
const long HOME_TRAVEL  = 6000;      // Give up after this much travel, about 135 deg
const long SWITCH_ZONE  = 90;        // A switch closing further down than this is a crash

struct Segment { int d[3]; unsigned long us; };
const byte QUEUE = 32;               // 320 ms of motion at 10 ms a segment
Segment queue[QUEUE], now;
byte head = 0, tail = 0;
bool running = false, homed = false, pending = false;
const char *fault = 0;
unsigned long started;
long pos[3] = {0, 0, 0};
int done[3];
char line[40];
byte length = 0;

bool pressed(byte pin) { return digitalRead(pin) == LOW; }
bool idle() { return !running && head == tail; }

void pulse(byte i, int sign) {
  digitalWrite(STEP[i], HIGH);
  delayMicroseconds(4);
  digitalWrite(STEP[i], LOW);
  pos[i] += sign;
}

void direction(byte i, bool down) {
  digitalWrite(DIR[i], down == DOWN_HIGH[i] ? HIGH : LOW);
  delayMicroseconds(5);                           // Direction setup before the next step
}

// Called on every pass of loop(): emit whatever steps are due by now
void runSegments() {
  if (!running) {
    if (head == tail) return;
    now = queue[tail];
    tail = (tail + 1) % QUEUE;
    for (byte i = 0; i < 3; i++) { direction(i, now.d[i] > 0); done[i] = 0; }
    started = micros();
    running = true;
  }
  unsigned long t = micros() - started;          // Not min(): it is a macro and would read the clock twice
  if (t > now.us) t = now.us;
  bool finished = true;
  for (byte i = 0; i < 3; i++) {
    int n = abs(now.d[i]);
    int due = (unsigned long)n * t / now.us;
    for (; done[i] < due; done[i]++) pulse(i, now.d[i] > 0 ? 1 : -1);
    if (done[i] < n) finished = false;
    if (pos[i] < 0 || (pressed(LIMIT[i]) && pos[i] > SWITCH_ZONE)) {   // Above the switch, or a crash
      fault = "limit"; running = false; homed = false; head = tail;
      return;
    }
  }
  if (finished) running = false;
}

// Raise every arm until its switch closes, all three together, then call that step 0
void home() {
  if (homed) {                                    // Back off, so the switch is met at the same speed every time
    for (byte i = 0; i < 3; i++) direction(i, true);
    for (long s = 0; s < HOME_BACKOFF; s++) {
      for (byte i = 0; i < 3; i++) pulse(i, 1);
      delayMicroseconds(1000000L / HOME_SPEED);
    }
  }
  for (byte i = 0; i < 3; i++) direction(i, false);
  bool at[3] = {false, false, false};
  homed = false;
  for (long s = 0; s < HOME_TRAVEL && !homed; s++) {
    homed = true;
    for (byte i = 0; i < 3; i++) {
      at[i] = at[i] || pressed(LIMIT[i]);
      if (!at[i]) { pulse(i, -1); homed = false; }
    }
    delayMicroseconds(1000000L / HOME_SPEED);
  }
  for (byte i = 0; i < 3; i++) pos[i] = 0;
  fault = homed ? 0 : "switch not found";
}

// Act on the line in `line`. False leaves it waiting, which holds the Orin back too
bool handle() {
  long a, b, c;
  unsigned long us;
  char text[64];
  switch (line[0]) {
    case 'S':
      if ((head + 1) % QUEUE == tail) return false;          // Queue full, try again on the next pass
      if (fault || !homed) Serial.println(fault ? "err fault" : "err not homed");
      else if (sscanf(line + 1, "%ld %ld %ld %lu", &a, &b, &c, &us) != 4 || us == 0) Serial.println("err bad segment");
      else {
        queue[head] = Segment{{(int)a, (int)b, (int)c}, us};
        head = (head + 1) % QUEUE;
        Serial.println("ok");
      }
      return true;
    case 'W':
      if (!idle()) return false;
      Serial.println(fault ? "err fault" : "ok");
      return true;
    case 'H':
      if (!idle()) return false;
      home();
      Serial.println(homed ? "ok" : "err switch not found");
      return true;
    case 'E':
      if (!idle()) return false;
      digitalWrite(MAGNET, line[2] == '1' ? HIGH : LOW);
      Serial.println("ok");
      return true;
    case '?':
      snprintf(text, sizeof text, "ok pos %ld %ld %ld sw %d%d%d btn %d homed %d", pos[0], pos[1], pos[2],
               pressed(LIMIT[0]), pressed(LIMIT[1]), pressed(LIMIT[2]), pressed(BUTTON), homed);
      Serial.println(text);
      return true;
  }
  Serial.println("err unknown command");
  return true;
}

void setup() {
  for (byte i = 0; i < 3; i++) {
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
    pinMode(LIMIT[i], INPUT_PULLUP);
  }
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  pinMode(MAGNET, OUTPUT);
  digitalWrite(MAGNET, LOW);
  pinMode(BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  runSegments();
  if (!pending && Serial.available()) {           // Read without blocking, the steps must keep coming
    char ch = Serial.read();
    if (ch == '\n') { line[length] = 0; length = 0; pending = true; }
    else if (ch != '\r' && length < sizeof line - 1) line[length++] = ch;
  }
  if (pending) pending = !handle();
}
