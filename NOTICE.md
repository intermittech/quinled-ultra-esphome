# Copyright and licence

Copyright (C) 2026 QuinLED.

These ESPHome components are licensed under the GNU General Public License, version 3 (`GPL-3.0`), the same as the
ESPHome C++ core they compile into. See [LICENSE](LICENSE).

## QuinLED Ultra protocol header and sender library (MIT)

`components/ultra_line/ultra_protocol.h`, `ultra_update.h`, `ultra_dport.h`, `ultra_transport.h` and the
`ultra_sender*.c` / `ultra_sender*.h` files are byte-identical copies from
[quinled-ultra-protocol](https://github.com/intermittech/quinled-ultra-protocol) (the CI pins the commit). They keep
their own licence, MIT (`SPDX-License-Identifier: MIT` in each file):

> MIT License
>
> Copyright (C) 2026 QuinLED
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
> (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
> publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
> so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
> MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
> FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
> CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Palettes in the pixel effect catalogues (colour stops as data)

`components/ur_digital_m/pixel_fx_catalogue.json` and `pixel_fx_catalogue_v407.json` list the receivers' effects and
palettes (the effects themselves run in the receiver firmware, whose NOTICE credits them); the first is a copy of
quinled-ultra-protocol's `catalogue/` (MIT, as above). The catalogues carry the palettes' colour stops and an `origin`
for each effect and palette. No code from WLED or FastLED is used here.

**Rainbow, Rainbow Bands, Party, Ocean, Forest, Lava, Heat and Cloud** are FastLED's colour palettes (colorpalettes.cpp:
RainbowColors_p, RainbowStripeColors_p, PartyColors_p, OceanColors_p, ForestColors_p, LavaColors_p, HeatColors_p,
CloudColors_p), MIT licence, Copyright (c) 2013 FastLED (Daniel Garcia and contributors).

**April Night, Orange & Teal and C9 New** are palettes by Aircoookie (Christian Schwinne) from WLED, added while WLED was
MIT-licensed (2018-2020; the same stops are in WLED v0.14.4, still MIT). Only their colour stops (numbers) are used,
resampled to sixteen entries. C9 New derives from the C9 colours of Mark Kriegsman's TwinkleFOX gist
(https://gist.github.com/kriegsman/756ea6dcae8e30845b5a). WLED's licence for these: MIT License, Copyright (c) 2016
Christian Schwinne (the same terms as the MIT text above).

FastLED's licence for its palettes: MIT License, Copyright (c) 2013 FastLED (the same terms as the MIT text above).

**Retro Clown** is the gradient "bhw1_01" by Blackheartedwolf (2011) from the cpt-city archive
(http://seaviewsensing.com/pub/cpt-city/bhw/bhw1/), licensed under the Creative Commons Attribution 3.0 licence
(https://creativecommons.org/licenses/by/3.0/), resampled to 16 entries (the colour values are the original's). It is
separately licensed data under CC BY 3.0, not part of the GPL-licensed code.

**Names.** Analogous, Hult, Magred and Yelmag follow the names in WLED's palette list, but their stops are not WLED's or
cpt-city's: they are our own look-alikes (the same mood and colour progression, stops chosen by eye). Every other
palette is our own work.
