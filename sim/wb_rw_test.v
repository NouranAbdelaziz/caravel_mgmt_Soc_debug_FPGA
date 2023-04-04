`default_nettype none

`timescale 1 ns / 1 ps

module wb_rw_test(
    input wire rx,
    output wire tx,
    input wire start
);

//baud 9600 bit time 104167

//parameter BTIME = 8680;
parameter BTIME = 104167;
//parameter BTIME = 7250;
parameter BITTIME = 104167;
//parameter BITTIME = 7250;

//2175 --> pass behavioural fail synth
//2165 --> pass behavioural fail synth
//2170 --> pass behavioural fail synth
//2173 --> pass behavioural fail synth
//2176 --> fail behavioural 
//2177 --> fail behavioural
//2180 --> fail behavioural 


reg TX;
wire RX;

reg [31:0] data;

assign RX = rx;
assign tx = TX;

initial begin
    TX <= 1'b1;
    //#10000;
    #150000;
    $display("Executing SRAM write");
    wb_write(30'h00400024, 32'h775555ab);
    //WB_WR(30'h00400024, 32'h775555ab);
    $display("Executing SRAM read");
    wb_read (32'h00400024, data);
    //WB_RD (32'h00400024, data);
    $display("output: %h", data);

    if (data != 32'h775555ab) begin
        $display("SRAM write failed");
        //$finish;
    end else begin
        $display("SRAM write succeeded");
    end
end

task uart_put(input [7:0] b);
    integer i;
    begin
        TX = 1'b0;
        #BTIME;
        for(i=0; i<8; i=i+1) begin
            TX = b[i];
            #BTIME;
        end
        TX = 1'b1;
        #(BTIME);
    end
endtask

task uart_put_ns(input [7:0] b);
    integer i;
    begin
        TX = 1'b0;
        #BTIME;
        for(i=0; i<8; i=i+1) begin
            TX = b[i];
            #BTIME;
        end
        TX = 1'b1;
//        #(BTIME);
    end
endtask


task uart_get(output [7:0] b);
    integer i;
    begin
        @(negedge RX);
        #BTIME;
        for(i=0; i<8; i=i+1) begin
            #(BTIME/2);
            b[i] <= RX;
            #(BTIME/2);
        end
        #(BTIME);
    end
endtask

task wb_read (input [31:0] addr, output [31:0] word);
    begin
        uart_put(8'h2);     // read cmd
        uart_put(8'h1);      // size
        uart_put(addr[31:24]);
        uart_put(addr[23:16]);
        uart_put(addr[15:8]);
        uart_put_ns(addr[7:0]);
        uart_get(word[31:24]);
        uart_get(word[23:16]);
        uart_get(word[15:8]);
        uart_get(word[7:0]);
    end
endtask

task wb_write (input [31:0] addr, input [31:0] word);
    begin
        uart_put(8'h1);     // write cmd
        uart_put(8'h1);      // size
        uart_put(addr[31:24]);
        uart_put(addr[23:16]);
        uart_put(addr[15:8]);
        uart_put(addr[7:0]);
        uart_put(word[31:24]);
        uart_put(word[23:16]);
        uart_put(word[15:8]);
        uart_put(word[7:0]);
    end
endtask

task UART_SEND (input [7:0] data);
    begin : task_body
        integer i;
        #BITTIME;
        //@(posedge HCLK);
        TX = 0;
        #BITTIME;
        for(i=0; i<8; i=i+1) begin
            TX = data[i];
            #BITTIME;
        end
        TX = 1;
        //#BITTIME;
    end
endtask

task UART_REC (output [7:0] data);
    begin : task_body
        integer i;
        @(negedge RX);
        #(BITTIME+(BITTIME/2));
        for(i=0; i<8; i=i+1) begin
            data[i] = RX;
            #BITTIME;
        end
        end
endtask

task WB_RD (input [31:0] addr, output [31:0] word);
    begin
        UART_SEND(8'h2);     // read cmd
        UART_SEND(8'h1);      // size
        UART_SEND(addr[31:24]);
        UART_SEND(addr[23:16]);
        UART_SEND(addr[15:8]);
        UART_SEND(addr[7:0]);
        UART_REC(word[31:24]);
        UART_REC(word[23:16]);
        UART_REC(word[15:8]);
        UART_REC(word[7:0]);
    end
endtask

task WB_WR (input [31:0] addr, input [31:0] word);
    begin
        UART_SEND(8'h1);     // write cmd
        UART_SEND(8'h1);      // size
        UART_SEND(addr[31:24]);
        UART_SEND(addr[23:16]);
        UART_SEND(addr[15:8]);
        UART_SEND(addr[7:0]);
        UART_SEND(word[31:24]);
        UART_SEND(word[23:16]);
        UART_SEND(word[15:8]);
        UART_SEND(word[7:0]);
    end
endtask

UART_MON MON (.RX(TX));

endmodule
