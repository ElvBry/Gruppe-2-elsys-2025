{
  "version": "1.2",
  "package": {
    "name": "",
    "version": "",
    "description": "",
    "author": "",
    "image": ""
  },
  "design": {
    "board": "go-board",
    "graph": {
      "blocks": [
        {
          "id": "26a95a27-a17b-4fbf-821d-3495f276cafb",
          "type": "basic.output",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD4",
                "value": "62"
              }
            ]
          },
          "position": {
            "x": 2504,
            "y": -40
          }
        },
        {
          "id": "e50403fb-6063-446f-8cf0-d24ac2f47023",
          "type": "basic.input",
          "data": {
            "name": "CLK_25MHz",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "CLK",
                "value": "15"
              }
            ],
            "clock": false
          },
          "position": {
            "x": -136,
            "y": 136
          }
        },
        {
          "id": "5a3aaad2-e2ef-495a-a9b7-3ea79f3f9a8a",
          "type": "basic.input",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD7",
                "value": "78"
              }
            ],
            "clock": false
          },
          "position": {
            "x": 2744,
            "y": 248
          }
        },
        {
          "id": "bec782e7-5f00-4ede-9d5a-aeffd78ee2d2",
          "type": "basic.output",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD3",
                "value": "63"
              }
            ]
          },
          "position": {
            "x": 1144,
            "y": 280
          }
        },
        {
          "id": "1c40b84f-570d-4e0d-9941-1b6f5ed16f82",
          "type": "basic.input",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD8",
                "value": "79"
              }
            ],
            "clock": false
          },
          "position": {
            "x": 2744,
            "y": 328
          }
        },
        {
          "id": "13dc8268-a515-43f0-ad5f-1f5d8ebb888b",
          "type": "basic.input",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD9",
                "value": "80"
              }
            ],
            "clock": false
          },
          "position": {
            "x": 2744,
            "y": 400
          }
        },
        {
          "id": "d3049f1d-c7d5-4411-9d08-0c2d21ec8e08",
          "type": "basic.input",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD10",
                "value": "81"
              }
            ],
            "clock": false
          },
          "position": {
            "x": 2744,
            "y": 480
          }
        },
        {
          "id": "4b9bedc8-b9e4-4646-ad99-136eabea6e62",
          "type": "basic.output",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD1",
                "value": "65"
              }
            ]
          },
          "position": {
            "x": 4808,
            "y": 784
          }
        },
        {
          "id": "ced78aae-6b58-4eb0-970f-3de436bb5461",
          "type": "basic.input",
          "data": {
            "name": "",
            "virtual": false,
            "pins": [
              {
                "index": "0",
                "name": "PMOD2",
                "value": "64"
              }
            ],
            "clock": false
          },
          "position": {
            "x": 2664,
            "y": 1496
          }
        },
        {
          "id": "327005d5-8437-42f1-ba97-f874b090663b",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "CLK_25MHz"
                },
                {
                  "name": "SCLK_ESP"
                }
              ],
              "out": [
                {
                  "name": "RST"
                }
              ]
            },
            "params": [],
            "code": "// Resets the block connected to the ESP32 for synchronisation\nlocalparam START = 2'b00;\nlocalparam WAITING = 2'b01;\nlocalparam RESET = 2'b10;\n\nreg [1:0] state = START;\nreg [1:0] next_state = START;\n\nreg out_rst = 0;\n\nreg [31:0] counter; \n\nalways @(*) begin\n    case(state)\n    START: next_state = SCLK_ESP ? WAITING : START;\n    WAITING: begin\n        if (SCLK_ESP)\n            next_state = (counter == 2500000) ? RESET : WAITING; //250 000 = 1 ms\n        else next_state = START;\n    end\n    RESET: next_state = (counter == 2500010) ? START : RESET; //10 cycles\n    default: next_state = START;\n    endcase\nend\n\nalways @(posedge CLK_25MHz) begin\n    case(state)\n    START: begin\n        out_rst <= 0;\n        counter <= 0;\n    end\n    WAITING: counter <= counter +1;\n    RESET: begin\n        out_rst <= 1;\n        counter <= counter +1;\n    end\n    endcase\n    state <= next_state;\nend\n\nassign RST = out_rst;\n"
          },
          "position": {
            "x": 2992,
            "y": 736
          },
          "size": {
            "width": 800,
            "height": 728
          }
        },
        {
          "id": "bb403957-92e6-464a-a156-786f26b3213a",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "SDO_BUFFER",
                  "range": "[47:0]",
                  "size": 48
                },
                {
                  "name": "CLK_25MHz"
                },
                {
                  "name": "SCLK_ESP"
                },
                {
                  "name": "RST"
                }
              ],
              "out": [
                {
                  "name": "ESP_SPI_SDO"
                },
                {
                  "name": "FIRST_VALUES",
                  "range": "[3:0]",
                  "size": 4
                }
              ]
            },
            "params": [],
            "code": "\nreg [63:0] sdo_buffer = 64'h0000100020003000;\nreg [6:0] counter = 63; // 0-63\n\nalways @(negedge SCLK_ESP or posedge RST) begin\n    if(RST) begin\n    sdo_buffer = 64'h0000100020003000;\n    counter <= 0; // 1 dummy bit since reset will clock out bit on negedge\n    end else begin\n        if(counter == 0) begin\n            //sdo_buffer <= 64'h0F001AD020C03B00;\n            sdo_buffer[63:60] <= 4'h0;\n            sdo_buffer[59:48] <= SDO_BUFFER[11:0];\n            sdo_buffer[47:44] <= 4'h1;\n            sdo_buffer[43:32] <= SDO_BUFFER[23:12];\n            sdo_buffer[31:28] <= 4'h2;\n            sdo_buffer[27:16] <= SDO_BUFFER[35:24];\n            sdo_buffer[15:12] <= 4'h3;\n            sdo_buffer[11:0]  <= SDO_BUFFER[47:36];\n            \n            counter <= 63;\n        end else counter <= counter - 1;\n    end\nend\n\nassign ESP_SPI_SDO = sdo_buffer[counter];\n\n"
          },
          "position": {
            "x": 3976,
            "y": 704
          },
          "size": {
            "width": 720,
            "height": 456
          }
        },
        {
          "id": "6dd5acb0-a673-468b-841e-508a609488f4",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "CLK_390kHz"
                }
              ],
              "out": [
                {
                  "name": "SDO_OUT_BUFFER_COS",
                  "range": "[47:0]",
                  "size": 48
                },
                {
                  "name": "DEBUG_PIN"
                }
              ]
            },
            "params": [],
            "code": "localparam START      = 2'b00;\r\nlocalparam GENERATING = 2'b01;\r\nlocalparam STOP       = 2'b10;\r\n\r\nreg [1:0] state = START;\r\nreg [1:0] next_state = START;\r\n\r\n\r\nreg [11:0] ref_cos[0:9];\r\ninitial begin\r\n   \r\n    ref_cos[0] = 12'd3277;  // 6554 / 2\r\n    ref_cos[1] = 12'd2964;  // 5928 / 2\r\n    ref_cos[2] = 12'd2145;  // 4290 / 2\r\n    ref_cos[3] = 12'd1133;  // 2266 / 2\r\n    ref_cos[4] = 12'd314;   // 628  / 2\r\n    ref_cos[5] = 12'd1;     // 2    / 2\r\n    ref_cos[6] = 12'd314;   // 628  / 2\r\n    ref_cos[7] = 12'd1133;  // 2266 / 2\r\n    ref_cos[8] = 12'd2145;  // 4290 / 2\r\n    ref_cos[9] = 12'd2964;  // 5928 / 2\r\nend\r\n\r\nreg [30:0] global_counter;\r\nreg [31:0] state_counter;\r\nreg [11:0] out_buffers[3:0];\r\nreg [3:0] reg_counter; \r\n\r\nalways @(*) begin\r\n    case(state)\r\n        START:      next_state = (global_counter == 10000) ? GENERATING : START;\r\n        GENERATING: next_state = (state_counter == 5000) ? STOP : GENERATING;\r\n        STOP:       next_state = START;\r\n        default:    next_state = START;\r\n    endcase\r\nend\r\n\r\n// Main state machine:\r\nalways @(posedge CLK_390kHz) begin\r\n    case(state)\r\n        START: begin\r\n            global_counter <= global_counter + 1;\r\n            // Reset our counters when in START\r\n            state_counter <= 0;\r\n            reg_counter   <= 0;\r\n        end\r\n\r\n        GENERATING: begin\r\n            // Increment the delay counter\r\n            state_counter <= state_counter + 1;\r\n            \r\n            // Cycle through cosine values\r\n            if(reg_counter == 9)\r\n                reg_counter <= 0;\r\n            else\r\n                reg_counter <= reg_counter + 1;\r\n\r\n            // Buffer activations:\r\n            // Buffer 0 is active immediately.\r\n            out_buffers[0] <= ref_cos[reg_counter];\r\n\r\n            // Buffer 1 activates after 30 cycles.\r\n            out_buffers[1] <= (state_counter >= 50) ? ref_cos[reg_counter] : 12'h000;\r\n            \r\n            // Buffer 2 activates after 60 cycles.\r\n            out_buffers[2] <= (state_counter >= 100) ? ref_cos[reg_counter] : 12'h000;\r\n            \r\n            // Buffer 3 activates after 90 cycles.\r\n            out_buffers[3] <= (state_counter >= 150) ? ref_cos[reg_counter] : 12'h000;\r\n        end\r\n\r\n        STOP: begin\r\n            // Reset everything on STOP\r\n            global_counter <= 0;\r\n            reg_counter    <= 0;\r\n            state_counter  <= 0;\r\n            out_buffers[0] <= 12'h000;\r\n            out_buffers[1] <= 12'h000;\r\n            out_buffers[2] <= 12'h000;\r\n            out_buffers[3] <= 12'h000;\r\n        end\r\n    endcase\r\n\r\n    // Update state at each clock edge.\r\n    state <= next_state;\r\nend\r\n\r\n// Drive the output bus from the buffers.\r\nassign SDO_OUT_BUFFER_COS[11:0]   = out_buffers[0];\r\nassign SDO_OUT_BUFFER_COS[23:12]  = out_buffers[1];\r\nassign SDO_OUT_BUFFER_COS[35:24]  = out_buffers[2];\r\nassign SDO_OUT_BUFFER_COS[47:36]  = out_buffers[3];\r\nassign DEBUG_PIN = (state[1:0] == GENERATING);\r\n"
          },
          "position": {
            "x": 1040,
            "y": 2456
          },
          "size": {
            "width": 824,
            "height": 1576
          }
        },
        {
          "id": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "SCLK"
                },
                {
                  "name": "SDO_READ"
                },
                {
                  "name": "SDO_IN1"
                },
                {
                  "name": "SDO_IN2"
                },
                {
                  "name": "SDO_IN3"
                },
                {
                  "name": "SDO_IN4"
                },
                {
                  "name": "RST"
                }
              ],
              "out": [
                {
                  "name": "SDO_OUT"
                },
                {
                  "name": "SDO_OUT_BUFFER",
                  "range": "[47:0]",
                  "size": 48
                }
              ]
            },
            "params": [],
            "code": "\n\nreg [11:0] sdo_in_buffer1 = 12'h000;\nreg [11:0] sdo_in_buffer2 = 12'h000;\nreg [11:0] sdo_in_buffer3 = 12'h000;\nreg [11:0] sdo_in_buffer4 = 12'h000;\nreg [47:0] sdo_out_buffer = 48'h000000000000;\n\nreg [4:0] counter = 0;\n\nreg sdo_in = 0;\n\nalways @(negedge SCLK) begin\n    if(counter == 12) begin\n        counter <= 0;\n        sdo_out_buffer[11:0] = sdo_in_buffer1[11:0];\n        sdo_out_buffer[23:12] = sdo_in_buffer2[11:0];\n        sdo_out_buffer[35:24] = sdo_in_buffer3[11:0];\n        sdo_out_buffer[47:36] = sdo_in_buffer4[11:0];\n    end else begin\n        sdo_in_buffer1[11-counter] <= SDO_IN1;\n        sdo_in_buffer2[11-counter] <= SDO_IN2;\n        sdo_in_buffer3[11-counter] <= SDO_IN3;\n        sdo_in_buffer4[11-counter] <= SDO_IN4;\n        counter <= counter +1;\n    end\nend\n\nassign SDO_OUT = sdo_in_buffer1[12-counter]&&SDO_READ;\nassign SDO_OUT_BUFFER[47:0] = sdo_out_buffer[47:0];"
          },
          "position": {
            "x": 2920,
            "y": 88
          },
          "size": {
            "width": 624,
            "height": 536
          }
        },
        {
          "id": "225f923e-2edc-407b-a64b-2d4bcdb9cc1b",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "CLK_25MHz"
                },
                {
                  "name": "RST"
                }
              ],
              "out": [
                {
                  "name": "CLK_390kHz"
                }
              ]
            },
            "params": [],
            "code": "reg [10:0] divcounter;\nalways @(posedge CLK_25MHz) begin\n    divcounter <= divcounter + 1;\nend\n  \nassign CLK_390kHz = divcounter[5]; \n"
          },
          "position": {
            "x": 176,
            "y": 208
          },
          "size": {
            "width": 768,
            "height": 208
          }
        },
        {
          "id": "6e953afd-6f9d-4805-bec4-0c6fdf7b6574",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "CNVST_IN"
                },
                {
                  "name": "CLK_25MHz"
                },
                {
                  "name": "RST"
                }
              ],
              "out": [
                {
                  "name": "T_EN"
                }
              ]
            },
            "params": [],
            "code": "localparam START = 1'b0;\nlocalparam DONE = 1'b1;\n\nreg state = START;\nreg next_state = START;\n\nreg t_en = 0;\n\nalways @(*) begin\n    case (state)\n    START: next_state = CNVST_IN ? START : DONE;\n    DONE: next_state = CNVST_IN ? START : DONE;\n    default: next_state = START;\n    endcase\nend\n \n\nalways @(posedge CLK_25MHz) begin\n    case(state)\n    DONE: t_en <= 1;\n    START: t_en = 0;\n    endcase\n    state <= next_state;\nend\n\nassign T_EN = t_en;"
          },
          "position": {
            "x": 1176,
            "y": -544
          },
          "size": {
            "width": 656,
            "height": 712
          }
        },
        {
          "id": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
          "type": "basic.code",
          "data": {
            "ports": {
              "in": [
                {
                  "name": "T_EN"
                },
                {
                  "name": "CLK_25MHz"
                },
                {
                  "name": "RST"
                }
              ],
              "out": [
                {
                  "name": "SCLK"
                },
                {
                  "name": "SDO_READ"
                }
              ]
            },
            "params": [],
            "code": "localparam START = 2'b00;\r\nlocalparam CLK = 2'b01;\r\nlocalparam DONE = 2'b10;\r\n\r\nreg [1:0] state = START;\r\nreg [1:0] next_state = START;\r\n\r\nreg clk_out;\r\nreg [10:0] counter = 0;\r\nreg sdo_read = 0;\r\n\r\nalways @(*) begin\r\n    case (state)\r\n    START: next_state = T_EN ? CLK : START;\r\n    CLK: next_state = (counter == 25) ? DONE : CLK; // 13 cycles\r\n    DONE: next_state = T_EN ? DONE : START;\r\n    default: next_state = START;\r\n    endcase\r\nend\r\n\r\nalways @(posedge CLK_25MHz) begin\r\n    case (state)\r\n    CLK: begin\r\n        counter <= counter +1;\r\n        clk_out <= ~clk_out;\r\n        sdo_read <= 1;\r\n    end\r\n    DONE: begin\r\n        counter <= 0;\r\n        clk_out <= 2'b00;\r\n        sdo_read <= 0;\r\n    end\r\n    endcase\r\n    state <=next_state;\r\nend\r\n\r\nassign SCLK = clk_out;\r\nassign SDO_READ = sdo_read;"
          },
          "position": {
            "x": 1912,
            "y": -128
          },
          "size": {
            "width": 528,
            "height": 664
          }
        }
      ],
      "wires": [
        {
          "source": {
            "block": "e50403fb-6063-446f-8cf0-d24ac2f47023",
            "port": "out"
          },
          "target": {
            "block": "225f923e-2edc-407b-a64b-2d4bcdb9cc1b",
            "port": "CLK_25MHz"
          },
          "vertices": []
        },
        {
          "source": {
            "block": "e50403fb-6063-446f-8cf0-d24ac2f47023",
            "port": "out"
          },
          "target": {
            "block": "6e953afd-6f9d-4805-bec4-0c6fdf7b6574",
            "port": "CLK_25MHz"
          },
          "vertices": [
            {
              "x": 1040,
              "y": 160
            }
          ]
        },
        {
          "source": {
            "block": "e50403fb-6063-446f-8cf0-d24ac2f47023",
            "port": "out"
          },
          "target": {
            "block": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
            "port": "CLK_25MHz"
          },
          "vertices": []
        },
        {
          "source": {
            "block": "225f923e-2edc-407b-a64b-2d4bcdb9cc1b",
            "port": "CLK_390kHz"
          },
          "target": {
            "block": "bec782e7-5f00-4ede-9d5a-aeffd78ee2d2",
            "port": "in"
          }
        },
        {
          "source": {
            "block": "225f923e-2edc-407b-a64b-2d4bcdb9cc1b",
            "port": "CLK_390kHz"
          },
          "target": {
            "block": "6e953afd-6f9d-4805-bec4-0c6fdf7b6574",
            "port": "CNVST_IN"
          },
          "vertices": []
        },
        {
          "source": {
            "block": "e50403fb-6063-446f-8cf0-d24ac2f47023",
            "port": "out"
          },
          "target": {
            "block": "bb403957-92e6-464a-a156-786f26b3213a",
            "port": "CLK_25MHz"
          },
          "vertices": [
            {
              "x": 2808,
              "y": 672
            },
            {
              "x": 3808,
              "y": 688
            }
          ]
        },
        {
          "source": {
            "block": "ced78aae-6b58-4eb0-970f-3de436bb5461",
            "port": "out"
          },
          "target": {
            "block": "bb403957-92e6-464a-a156-786f26b3213a",
            "port": "SCLK_ESP"
          },
          "vertices": [
            {
              "x": 3848,
              "y": 1184
            }
          ]
        },
        {
          "source": {
            "block": "e50403fb-6063-446f-8cf0-d24ac2f47023",
            "port": "out"
          },
          "target": {
            "block": "327005d5-8437-42f1-ba97-f874b090663b",
            "port": "CLK_25MHz"
          },
          "vertices": [
            {
              "x": 2080,
              "y": 896
            },
            {
              "x": 2136,
              "y": 904
            }
          ]
        },
        {
          "source": {
            "block": "327005d5-8437-42f1-ba97-f874b090663b",
            "port": "RST"
          },
          "target": {
            "block": "bb403957-92e6-464a-a156-786f26b3213a",
            "port": "RST"
          }
        },
        {
          "source": {
            "block": "ced78aae-6b58-4eb0-970f-3de436bb5461",
            "port": "out"
          },
          "target": {
            "block": "327005d5-8437-42f1-ba97-f874b090663b",
            "port": "SCLK_ESP"
          },
          "vertices": [
            {
              "x": 2776,
              "y": 1528
            }
          ]
        },
        {
          "source": {
            "block": "bb403957-92e6-464a-a156-786f26b3213a",
            "port": "ESP_SPI_SDO"
          },
          "target": {
            "block": "4b9bedc8-b9e4-4646-ad99-136eabea6e62",
            "port": "in"
          }
        },
        {
          "source": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_OUT_BUFFER"
          },
          "target": {
            "block": "bb403957-92e6-464a-a156-786f26b3213a",
            "port": "SDO_BUFFER"
          },
          "size": 48
        },
        {
          "source": {
            "block": "5a3aaad2-e2ef-495a-a9b7-3ea79f3f9a8a",
            "port": "out"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_IN1"
          }
        },
        {
          "source": {
            "block": "1c40b84f-570d-4e0d-9941-1b6f5ed16f82",
            "port": "out"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_IN2"
          }
        },
        {
          "source": {
            "block": "13dc8268-a515-43f0-ad5f-1f5d8ebb888b",
            "port": "out"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_IN3"
          }
        },
        {
          "source": {
            "block": "d3049f1d-c7d5-4411-9d08-0c2d21ec8e08",
            "port": "out"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_IN4"
          }
        },
        {
          "source": {
            "block": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
            "port": "SDO_READ"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SDO_READ"
          },
          "vertices": [
            {
              "x": 2528,
              "y": 200
            }
          ]
        },
        {
          "source": {
            "block": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
            "port": "SCLK"
          },
          "target": {
            "block": "ac4c1c6f-6c1b-4fca-b5fc-e95b45ac1a15",
            "port": "SCLK"
          }
        },
        {
          "source": {
            "block": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
            "port": "SCLK"
          },
          "target": {
            "block": "26a95a27-a17b-4fbf-821d-3495f276cafb",
            "port": "in"
          }
        },
        {
          "source": {
            "block": "6e953afd-6f9d-4805-bec4-0c6fdf7b6574",
            "port": "T_EN"
          },
          "target": {
            "block": "f4afb87f-256e-4df0-a0bc-675aa43cea2b",
            "port": "T_EN"
          }
        }
      ]
    }
  },
  "dependencies": {}
}
