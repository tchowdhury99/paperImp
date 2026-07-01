You are working inside my project directory:  
/home/tchowdh4/paperImp/  
This project is an STL implementation related to the paper:  
“Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles” by Choi et al.  
Important files likely present in this folder:  
\- /home/tchowdh4/paperImp/STL\_FINAL\_REPORT.md  
\- /home/tchowdh4/paperImp/STL\_COMPLETION\_STATUS.md  
\- /home/tchowdh4/paperImp/STL\_IMPLEMENTATION\_GUIDE.md  
\- /home/tchowdh4/paperImp/SbRRfSAoRV.pdf  
\- /home/tchowdh4/paperImp/offline\_stl\_baro.py  
\- /home/tchowdh4/paperImp/offline\_stl\_gps.py  
\- /home/tchowdh4/paperImp/offline\_stl\_gyro.py  
\- /home/tchowdh4/paperImp/offline\_stl\_multi\_sensor.py  
\- /home/tchowdh4/paperImp/offline\_stl\_baro\_persistent.py  
\- /home/tchowdh4/paperImp/offline\_stl\_baro\_recovery.py  
\- /home/tchowdh4/paperImp/offline\_stl\_any\_attack\_recovery.py  
\- /home/tchowdh4/paperImp/offline\_stl\_altitude\_bounds.py

STRICT POLICY:  
Do not deviate from the paper.  
Do not optimize.  
Do not invent better thresholds.  
Do not add new sensors.  
Do not add new attacks.  
Do not improve the method beyond the paper.  
Do not change the dataset unless absolutely required.  
Do not use the previous V9/software-sensor project.  
Stay strictly within Choi et al.’s paper and the existing STL implementation guide.

Main correction needed:  
The current report/scripts may incorrectly explain residual formulas using variables like:

BARO\_Alt\_attacked  
GPS\_North\_attacked  
GPS\_East\_attacked  
GyrX\_attacked

directly as if they are the theoretical paper-level residual variables.  
This must be corrected.  
According to the paper, especially Algorithm 1 Runtime Recovery Monitoring, the residual is based on:

m \= actual physical sensor measurement received by the monitor/controller  
ms \= software sensor prediction  
The paper computes:  
r ← r \+ |m − ms|

Therefore, all theoretical residual equations must be written as:  
sensor\_error(t) \= |m\_sensor(t) − ms\_sensor(t)|  
and, if using the paper’s accumulated residual form:  
R\_sensor,N(t) \= Σ |m\_sensor(i) − ms\_sensor(i)| over the selected window  
The attacked variables must only be described as offline attack-simulation variables that corrupt m\_sensor(t) during the attack window. They are not the theoretical residual variables.  
Correct conceptual explanation:  
During normal time:  
m\_sensor(t) \= clean real physical sensor measurement  
During simulated attack time:  
m\_sensor(t) \= artificially corrupted physical sensor measurement  
But the formula must always remain:  
residual \= |m\_sensor(t) − ms\_sensor(t)|  
or, paper-aligned accumulated residual:  
r ← r \+ |m − ms|  
For barometer:  
Correct theory:  
m\_baro(t) \= real barometer measurement  
ms\_baro(t) \= software/model barometer prediction  
baro\_error(t) \= |m\_baro(t) − ms\_baro(t)|

For my dataset approximation:  
m\_baro(t) \= BARO\_Alt(t)  
ms\_baro(t) \= alt(t)

Offline attack simulation only:  
m\_baro(t) is corrupted during attack window, for example by adding \+3.0 m.

Do NOT write the theoretical formula as:  
baro\_residual(t) \= |BARO\_Alt\_attacked(t) − alt(t)|

Instead write:  
baro\_residual(t) \= |m\_baro(t) − ms\_baro(t)|

Then separately explain:  
For offline attack simulation, m\_baro(t) was generated from BARO\_Alt(t) and corrupted during the attack window.

Tasks:

1\. First, inspect all relevant markdown reports and Python scripts.  
Search for:  
\- BARO\_Alt\_attacked  
\- GPS\_North\_attacked  
\- GPS\_East\_attacked  
\- GyrX\_attacked  
\- attacked  
\- residual  
\- baro\_res  
\- gps\_res  
\- gyro\_res  
\- m  
\- ms  
\- r ← r \+ |m  
\- threshold  
\- STL formula

2\. Make a backup before modifying anything.  
Create a folder:  
/home/tchowdh4/paperImp/stl\_correction\_backup/

Copy the original markdown reports and original offline\_stl\_\*.py scripts into that backup folder before editing.

3\. Correct the markdown reports.  
At minimum, correct:  
\- STL\_FINAL\_REPORT.md  
\- STL\_COMPLETION\_STATUS.md if it contains the same issue

The reports must clearly separate:  
A. Paper-level residual formula  
B. Dataset approximation  
C. Offline attack simulation

Use this format where appropriate:

Paper-level:  
e\_sensor(t) \= |m\_sensor(t) − ms\_sensor(t)|

Paper accumulated residual:  
r(t) \= r(t−1) \+ |m\_sensor(t) − ms\_sensor(t)|

Windowed form:  
R\_sensor,N(t) \= Σ from i=t−N+1 to t of |m\_sensor(i) − ms\_sensor(i)|

Attack simulation note:  
The attacked variable is only used to corrupt m\_sensor(t) during the attack window. It is not part of the theoretical paper equation.

4\. Correct the Python scripts if needed.  
The code should use paper-aligned variable naming where possible.

Example correction for barometer:

Instead of this style:  
BARO\_Alt\_attacked \= BARO\_Alt.copy()  
BARO\_Alt\_attacked\[attack\_start:attack\_end\] \+= 3.0  
baro\_residual \= np.abs(BARO\_Alt\_attacked \- alt)

Use this style:  
m\_baro \= BARO\_Alt.copy()  
ms\_baro \= alt.copy()

\# Offline attack simulation only: corrupt the physical measurement m\_baro.  
m\_baro\[attack\_start:attack\_end\] \+= 3.0

baro\_error \= np.abs(m\_baro \- ms\_baro)

If the implementation is using instantaneous residual because the STL guide explicitly required that, keep the existing STL threshold structure but rename/explain it as instantaneous error:  
baro\_error(t) \= |m\_baro(t) − ms\_baro(t)|

If the paper requires cumulative residual for a corrected/paper-aligned version, implement the windowed residual separately and clearly:  
R\_baro\_N(t) \= sum of baro\_error over the current window

Do not silently change the scientific meaning. If you change instantaneous residual to cumulative residual, explicitly document why this is required by Algorithm 1\.

5\. Correct each formula group:

Barometer Integrity:  
Correct residual:  
baro\_error(t) \= |m\_baro(t) − ms\_baro(t)|

GPS Integrity:  
gps\_north\_error(t) \= |m\_gps\_north(t) − ms\_gps\_north(t)|  
gps\_east\_error(t) \= |m\_gps\_east(t) − ms\_gps\_east(t)|

Gyroscope Integrity:  
gyro\_error\_x(t) \= |m\_gyr\_x(t) − ms\_gyr\_x(t)|  
gyro\_error\_y(t) \= |m\_gyr\_y(t) − ms\_gyr\_y(t)|  
gyro\_error\_z(t) \= |m\_gyr\_z(t) − ms\_gyr\_z(t)|

Multi-Sensor Compound Spec:  
Keep the same STL structure if guide/paper requires it, but make sure each residual/error variable is defined using m\_sensor and ms\_sensor, not attacked variables.

Persistent Barometer Attack Pattern:  
Use baro\_error or windowed baro residual based on the corrected paper-aligned definition.

Barometer Recovery Within 10 s:  
Use corrected baro\_error/residual definition.

Multi-Sensor Any-Attack Recovery:  
Use corrected baro and gyro residual definitions:  
φ\_any\_attack \= (baro\_residual \> ε\_baro) or (gyro\_residual\_x \> ε\_gyr)  
where each residual is derived from |m − ms|.

Altitude Bounds:  
This one is not a sensor residual formula. Do not force attacked/real sensor notation into it. Keep:  
G\[0:580ms\] ((alt \> 0.97) and (alt \< 29.70))  
unless the guide explicitly says otherwise.

6\. Verify the STL formulas.  
Check whether the STL formulas themselves need correction because of the residual correction.  
Do not change STL formulas unnecessarily.  
Only change them if the existing formula is inconsistent with the paper’s Algorithm 1 or the guide.

Important:  
If the current STL formula says:  
G\[0:580ms\] (baro\_residual \< 0.30)

Then verify whether baro\_residual means:  
A. instantaneous error |m − ms|  
or  
B. accumulated/windowed residual Σ|m − ms|

The paper’s Algorithm 1 uses accumulated residual r.  
If the guide simplified this to instantaneous residual, document this as a guide-based simplification.  
Do not hide this issue.

7\. Run all corrected scripts using the required interpreter:

/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Run:  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_baro.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_gps.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_gyro.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_multi\_sensor.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_baro\_persistent.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_baro\_recovery.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_any\_attack\_recovery.py  
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 /home/tchowdh4/paperImp/offline\_stl\_altitude\_bounds.py

8\. Verify output plots still exist and are not empty:  
test \-s /home/tchowdh4/paperImp/stl\_result\_baro.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_gps.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_gyro.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_multi\_sensor.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_baro\_persistent.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_baro\_recovery.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_any\_attack\_recovery.png  
test \-s /home/tchowdh4/paperImp/stl\_result\_altitude\_bounds.png

9\. Create a new markdown file:

/home/tchowdh4/paperImp/STL\_PAPER\_ALIGNMENT\_CORRECTION\_REPORT.md

This file must include:

\# STL Paper Alignment Correction Report

Sections:  
1\. What was wrong  
2\. Why it was wrong according to Choi et al.  
3\. Correct residual equation from the paper  
4\. Difference between m\_sensor and attacked simulation variable  
5\. Corrected equations for barometer, GPS, gyroscope  
6\. Corrected STL formulas, if any were changed  
7\. Scripts modified  
8\. Reports modified  
9\. Backup location  
10\. Verification commands and outputs  
11\. Remaining deviations, if any  
12\. Final status

The report must clearly state:  
\- Whether the Python scripts were only renamed/restructured or mathematically changed.  
\- Whether any STL formulas changed.  
\- Whether instantaneous residuals are still used.  
\- Whether cumulative/windowed residuals were implemented.  
\- Whether this matches the paper or is a guide-based simplification.  
\- Whether any remaining deviation exists.

10\. After finishing, print:  
\- The list of modified files  
\- The list of generated files  
\- The exact equations after correction  
\- Whether all scripts ran successfully  
\- Whether any STL formula was changed  
\- Whether there are any remaining deviations from the paper

Remember:  
The purpose is not to improve the result.  
The purpose is to correct the implementation and documentation so the equations match the paper.  
Stay strictly inside the paper and the existing guide.  
Do not deviate.