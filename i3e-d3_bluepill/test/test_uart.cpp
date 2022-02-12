#include <mbed.h>

Serial rasp(PA_9, PA_10, 115200);
DigitalOut led(PC_13);


int main() {
  char pacote_recebido[6];
  int v_lin, v_ang;
  while(1) {
    if (rasp.readable()) {
      if (rasp.getc() == 'v') {
        for(int i=0; i<6; i++)
          pacote_recebido[i] = rasp.getc();
      }
      v_lin = (pacote_recebido[0] - '0')*100 + (pacote_recebido[1] - '0')*10 + (pacote_recebido[2] - '0');
      v_ang = (pacote_recebido[3] - '0')*100 + (pacote_recebido[4] - '0')*10 + (pacote_recebido[5] - '0');
      rasp.printf("v%03d%03d", v_lin, v_ang);
    }

  }
}
