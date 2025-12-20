# MIDI Control Change (CC) Mapping for Venus Effect

| Parameter | MIDI CC | Value Range | Description |
|-----------|---------|-------------|-------------|
| **Decay** | 14 | 0-127 | Reverb Decay Time |
| **Mix** | 15 | 0-127 | Dry/Wet Mix (0=Dry, 127=Wet) |
| **Damp** | 16 | 0-127 | High Frequency Damping |
| **Shimmer** | 17 | 0-127 | Shimmer Amount |
| **Shimmer Tone** | 18 | 0-127 | Shimmer Filter/Tone |
| **Detune** | 19 | 0-127 | Detune Amount (Middle=Neutral if bipolar, currently 0-1 mapped) |
| **Shimmer Mode** | 20 | 0-2 | 0=Down, 1=Up, 2=Both (Split 0-42, 43-84, 85-127) |
| **Reverb Mode** | 21 | 0-2 | 0=Less Lofi, 1=Normal, 2=More Lofi |
| **Drift Mode** | 22 | 0-2 | 0=Slow, 1=None, 2=Fast |
| **Freeze** | 23 | 0 or 127 | 0=Off, >64=On |
| **Bypass** | 24 | 0 or 127 | 0=Active, >64=Bypass |
