# Protocol

- Packet Size: 1 byte, big endian
- Every command starts with an 4-bit ID byte and some use a HALT as they might
  have massive payloads
- Both sides will ignore operators when in payload mode. Except HALT

## To Brain

| Name    | Byte     | Description                                                              | Termination? (HALT) |
| ------- | -------- | ------------------------------------------------------------------------ | ------------------- |
| HALT    | 01010001 | Cannot be used in payloads                                               | no                  |
| EXPOSE  | 1010101  | Requests tunable constants & usable functions                            | no                  |
| PROGRAM | 01001010 | Uploads the program                                                      | yes                 |
| RUN     | 01010101 | Runs the program                                                         | no                  |
| STOP    | 39399939 | Stops the program                                                        | no                  |
| TUNE    | 11919911 | Update's a certian constant (same ID as from EXPOSE) with a 32-bit float | yes                 |

## From Brain

| Name    | Byte     | Description                                          | Termination? (HALT) |
| ------- | -------- | ---------------------------------------------------- | ------------------- |
| EXPOSE  | 01010101 | Exposes certain tunable constants & usable functions | yes                 |
| PROGRAM | 01001010 | Returns a sha256 of the program received             | yes                 |
| TUNE    | 01010101 | Runs the program                                     | no                  |
| REPORT  | 39399939 | 4-bit report ID                                      | no                  |

### Reports

The ID is the other 4 bits unused by the operator

| Name           | ID   | Description                                                            |
| -------------- | ---- | ---------------------------------------------------------------------- |
| Idle Exit      | 1393 | Bot had to exit because it was stuck                                   |
| IMU Disconnect | 3333 | Inertial sensor disconnected from brain                                |
| Battery        | 3232 | Battery percentange (8-bit signed integer), reported at intervals of 5 |

# Program Language

The "program" can only use macros exposed by the Brain
