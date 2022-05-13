#include "mbed.h"

double x,y;
double x2,y2,L12,L22,xy,xy2;
double teta1,teta2;
double L1=1.3,L2=1.75;
double ew,ez,ey,ek,el;
double a,b,c,d,e,e2,f,g,o;
float radtodeg;

Serial stm(USBTX,USBRX);

int main() {
		stm.baud(9600);
    x=0;//cos(45*3.14/180);
		y=3;//sin(45*3.14/180);
	
		radtodeg=180/3.14;
		x2 = x*x;
    y2 = y*y;
	
    xy=sqrt(x2+y2);
		xy2=xy*xy;
		L12=L1*L1;
    L22=L2*L2;
		
		e = (acos(L22-L12-xy2))/(-2*L12*xy);
		if(e <= 0){
			e = 0;
			ek = e;
		}
		else{
			e=e;
		}
		ey = atan2(y,x);
		
		ew = (acos(xy2-L12-L22))/(-2*L1*L2);
		
		if(ew<=0){
			ew = 0;
			el = ew;
		}
		else{
			ew=ew;
		}
	
	  ez = ey-ek;
		
		if(ez<=0){
			ez = 0;
		}
		else{
			ez=ez;
		}
	while(1){
    teta1= (ez)*radtodeg ;
		teta2= 180-(radtodeg*el);
		stm.printf("teta1 = %f teta2 = %f \n \r",teta1,el);
		
		
		//stm.printf("T1 = %f , T2 = %f \n \r",teta1,teta2);
	}
    
}
