# LOGC
## Logic gate

**Description:**  *Multi-pixel logic gate. Use PSCN / COPR for input, NSCN / ZINC for output. Tmp = type, pavg0 = output voltage.*

Any connected blob of LOGC is considered a single chip. Note that the entire blob should have the same tmp, doing so otherwise 
may result in undefined behavior. LOGC melts at 2300 C, with a default output voltage of 5 V.

Set the tmp to set the type of logic gate:

```
AND = 0
OR = 1
NOT = 2
XOR = 3
NAND = 4
NOR = 5
XNOR = 6
```

If the tmp is not in the list above it will default to 0 (AND).

Inputs are done with PSCN / COPR wire (must be directly adjacent to a LOGC). Voltages with magnitude >= 0.7 V are considered "true", 
whereas voltages otherwise are "false". The NOT gate will only invert the 1st input, other gates can take any number of inputs (Note that 
having no inputs is considered undefined behavior).

If the result is true, will output pavg0 to any NSCN / ZINC directly adjacent to the same group of LOGC, otherwise outputs 0.

Like all chips, LOGC does not draw any current.

## Technical
#### Properties

**tmp:** Type of chip, see above

**tmp2:** Internal chip id, automatically assigned

**pavg[0]:** Voltage to output when true
