# teclado
Pequeno teclado dividido, impresso em 3D

Hardware e software para um teclado dividido em duas metades de 18 teclas cada.

As teclas operam magneticamente, com 3 pequenos ímãs em cada.
O movimento de cada tecla é convertido por um sensor hall analógico.

Cada metade tem um microcontrolador (RP2040); eles se comunicam por serial entre si; um deles se comunica com o host por USB.

O software é escrito em C, usando o SDK da Raspberry Pi e tiny USB.

O layout do teclado é inspirado no Miryoku.
