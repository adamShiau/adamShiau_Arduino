```mermaid
graph LR
  ROOT[HINS_MCU_V1.0] --> SRC[src]
  ROOT --> APP[HINS_MCU_V1.0.ino]

  subgraph SRC
    ACQ[Acquisition]
    AHRS[AHRS]
    COMM[Communication]
    OUTPUT[Output]
    CFG[Config/Params]
    SYS[System/Common]
  end
  ```