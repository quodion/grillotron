# grillotron
Grillotron ESP8266 based simple PID grill / smoker temperature monitoring and control system. It's a practical way for me to learn arduino / ESP8266 hardware and software development

## structure
grillotron connects to **inputs** (TC probes), **outputs** (currently: blower) and uses basic **PID** algorhitms (under development) to 
compute outputs towards **targets**. 

## Program
The program currently uses **arrays** dimensioned by 2 constants (maxprobes and maxoutputs) that include **probe temperatures (target, current, past)**, **PID parameters** and **output parameters**. **Inputs** contain a single identifier for related **outputs** which theoretically would allow you to use a single grillotron to cover multiple smokers / grills.

## Probes
The program currently supports **6 probes** that can be designated as food probes or environmental temperature probes. They're connected through **MAX6675** modules. 
