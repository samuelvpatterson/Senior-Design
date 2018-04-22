#ifndef BUTTONS_32_H
#define BUTTONS_32_H

extern double error;
extern double target_error;
extern int bit_number;
extern int trim_status;
extern int trim_algo;

void EdgeCounter_Init(void);

int error_return(void);
int Bit_numbers_active(void);
int Trim_algo(void);
int Trim_algo_status(void);



void Read_Input(void);
void Button_Input(int button_in);

void GPIOPORTF_Handler(void); //Will check in between 10ms like it says in book

#endif
