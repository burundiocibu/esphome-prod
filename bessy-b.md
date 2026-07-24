# bessy-b
A replacement controller for a pitboss pellet grill

Frist problem: phantom input from the encoder
1. Flyback diode across the auger relay coil if it doesn't have one already — this is very likely your actual root cause, since the glitch lands right on the relay's turn-off transient.
2. RC filter on each encoder line (e.g. 100Ω series + 0.1µF to GND at the ESP32 pin) — slews the edge past the noise pulse width without meaningfully affecting detent-speed rotation.

Second problem, Using the ads to sample the PTD. Use a NAU7802 or such

Third prob; display broken?

Forth prob: current sensor is on auger, not igniter.

5th prob: PTD needs to be mounted in the lid or maybe just under the grate in the back

cd ~/git/esphome/config
claude --resume c9dfa441-d66a-4cd4-b537-9891366a810b


## PID Tuning

Running it:

Light and stabilize at your target (say 225°F) under the existing PID. Watch augers_heating_on_time in the logs once it's holding — on_time / 15 is your steady hold duty.
Set duty HIGH ~0.10–0.15 above that hold duty and duty LOW ~0.10–0.15 below it, both inside [0.05, 0.80], both keeping the fire alive. That centers the swing on the real operating point, which is what makes the extracted Ku valid there. d = (HIGH − LOW)/2 is the relay amplitude the math uses.
Flip PID Autotune on. It bypasses the PID and relays the auger around setpoint; temp oscillates. After 2 discarded + 4 averaged cycles it logs pb / ti / td, turns itself off, and the PID resumes.
Type the three numbers into pid_pb / pid_ti / pid_td.

A few things that'll bite if ignored: if the temp sits entirely above or below setpoint and never crosses, your HIGH/LOW aren't centered — nudge them (live, via the numbers) and restart. Keep the lid shut with a representative meat load and no wild weather so the tune reflects a real cook. And note the applied duty only updates when the auger loop next reads the globals (≤15 s), while the detector samples at 2 s — negligible phase lag on a multi-minute period, but drop tune_cycle_s if you want it tighter.


In the esphome device bessy-b.yaml I have a pid controller for a pellet grill. I need to tune it with an Åström–Hägglund
 algorythm. Could you add the code to do so?


  What I added:
 - Åström–Hägglund relay autotune state in globals:
 - HA-exposed tune controls:
     - Autotune duty HIGH
     - Autotune duty LOW
     - Autotune hysteresis (F)
 - A template switch:
     - PID Autotune (relay)
 - A 2-second interval: loop that:
     - relays the auger duty around the setpoint
     - measures limit-cycle period/amplitude
     - computes Ku and Tu
     - logs suggested pid_pb, pid_ti, pid_td
 - PID reset handling so normal PID resumes cleanly after autotune
 - Guarding so sc_pot_heating stops calling sc_pid_control while autotune is active

 Validation:
 - Ran: ./.venv/bin/esphome config bessy-b.yaml
 - Result: INFO Configuration is valid!

 How to use it:
 1. Start the smoker and let it stabilize near your target temp.
 2. Note steady hold duty:
     - hold duty ≈ augers_heating_on_time / 15
 3. Set:
     - Autotune duty HIGH ≈ hold duty + 0.10 to 0.15
     - Autotune duty LOW ≈ hold duty - 0.10 to 0.15
     - keep both within 0.05 .. 0.80
 4. Turn on PID Autotune (relay).
 5. Watch logs for:
     - ==== AUTOTUNE COMPLETE ====
     - suggested pb, ti, td
 6. Copy those values into:
     - PID Proportional Band
     - PID Ti (Reset Time)
     - PID Td (Rate Time)

     [14:57:11.017][S][switch]: 'PID Autotune (relay)' >> ON
[15:10:04.973][I][tune:1378]: warmup 1 discarded: period=613.3s amp=11.68F
[15:22:26.516][I][tune:1378]: warmup 2 discarded: period=741.5s amp=18.01F
[15:33:43.129][I][tune:1376]: cycle 1: period=676.6s amp=18.45F
[15:44:57.463][I][tune:1376]: cycle 2: period=674.3s amp=14.24F
[15:57:14.735][I][tune:1376]: cycle 3: period=737.3s amp=11.83F
[16:08:08.815][I][tune:1376]: cycle 4: period=654.1s amp=19.41F
[16:08:08.821][I][tune:1409]: ==== AUTOTUNE COMPLETE ====
[16:08:08.822][I][tune:1410]: Ku=0.01014 duty/F  Tu=685.6s  a=15.98F  d=0.125  h=3.00F
[16:08:08.822][I][tune:1411]: Tyreus-Luyben ->  pb=315.6   ti=1508.3   td=108.82
[16:08:08.829][I][tune:1412]: Type those into pid_pb / pid_ti / pid_td.
[16:08:08.829][I][tune:711]: Autotune stopped; PID resumes.
[16:08:09.043][S][switch]: 'PID Autotune (relay)' >> OFF