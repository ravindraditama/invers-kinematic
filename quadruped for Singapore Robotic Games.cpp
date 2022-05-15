#include "mbed.h"
#include "servo.h"
#include "MPU9250.h"

#define dt 0.01
#define degConvert 57.295776
#define phi 3.14159254

Timer timer;
MPU9250 mpu9250;

const int constKalibrasi=335,
					spRed_sebelumSandune=-51,
					spRed_sesudahSandune=-66,
					spRed_BaratSesudahSandune=-12,
					spRed_TabrakSesudahSandune=-53,

					spBlue_sebelumSandune=-83,
					spBlue_sesudahSandune=-66,
					spBlue_TimurSesudahSandune=0,
					spBlue_TabrakSesudahSandune=0;

const float konstanta_belok=0.5;

int	length,
		verified,
		bantu,
		temporary_date,
		last_variabel_left,
		last_variabel_right,
		filterUltrasonik[10],
		filterCompass[500];

int whistleValue, whistleValueLast, value, nilaiPeluit, STEP;

long	variabel_jumlah,
			variabel_ultra;

char data[5], 
		 mode[5],
		 mode_2[5];

float legConstUukhai=0,
			COMPASS,
			frontLegConst=8,
			counting,
			counting_2,
			counting_3,
			jarak_bawah,
			Ex, Ey, Ez,
			Nx, Ny, Nz,
			teta1, teta2, teta3, 
			teta1_2, teta2_2, teta3_2,
			taw, dho, alfa, betaa,
			l1 = 13, l2 = 16, l3 = 7.5, //PANJANG KAKI
			l1_grip = 16, l2_grip = 14,	//PANJANG GRIPPER
			line_position, y_spesial,
			poster1, poster2,
			poster3, poster4,
			sum, jumlah,
			vectorDotResult1, 
			vectorDotResult1A, 
			vectorDotResult2,
			vectorDotResult2A,
			tempX, tempY, tempZ, 
			heading;

double	compAngleX,
				compAngleY,
				compAngleZ,

				gyroXrate,
				gyroYrate,
				gyroZrate,

				tempMinX=-32767, tempMinY=-32767, tempMinZ=-32767,
				tempMaksX=32767, tempMaksY=32767, tempMaksZ=32767;

uint32_t sumCount = 0;

Serial stmpc(USBTX, USBRX, 115200);
			 //stmrasp(NC, NC, 115200);

DigitalIn peluit(PA_12);

Servo kib1(PC_7), //KIRI BELAKANG
			kib2(PB_3),
			kib3(PB_6),

			kab1(PB_7), //KANAN BELAKANG pb7
			kab2(PA_5),
			kab3(PB_10), //pb_10

			kad1(PB_8), //KANAN DEPAN
			kad2(PC_9),
			kad3(PB_9),

			kid1(PC_6), //KIRI DEPAN
			kid2(PC_8),
			kid3(PA_11),

			A(PB_13),
			B(PA_10),
			C(PB_14); //PB_0

DigitalIn userButton(PC_13),			//USER BUTTON
					echoPin(PD_2),					//ECHO ULTRASONIK
					infrared_gerege(PC_12), //INFRARED GEREGE
					infrared_uukhai(PC_10);	//INFRARED UUKHAI

DigitalOut trigPin(PC_11); //TRIGGER ULTRASONIK

void date_received();
void invers(double x, double y, double z);
void invers_2dof(double x, double y);
void invers_3dof(double x, double y, double z);
void invers_gripper(double x, double y);
void rotasi_kiri(float panjang, float jml_ulang);
void rotasi_kanan(float panjang, float jml_ulang);
void vectorDot1(float A, float B, float C);
void vectorDot1A(float A, float B, float C);
void vectorDot2(float A, float B, float C);
void vectorDot2A(float A, float B, float C);
void averageFilter(int a, int jmlh_ulang);

//===============================SERVO CALIBRATION & CALCULATION================================//
void tulis(int a,int b,int c, //kiri blkng 
					 int d,int e,int f, //kanan dpn
					 int g,int h,int i, //kanan blkng
					 int j,int k,int l){ //kiri dpn					  

	kib1.position(a+16); //PC_7  // KIRI BELAKANG Z 
	kib2.position(b-6); //PB_3
	kib3.position(c-74); //PB_6 -16
						 
	kad1.position(d-8); //PB_8  // KANAN DEPAN Z
 	kad2.position(e-2); //PC_9 
	kad3.position(f-66); //PB_9 -39

	kab1.position(g+17); //PB_7	// KANAN BELAKANG Z
	kab2.position(h+13); //PA_5
	kab3.position(i-54); //PB_10  -2

	kid1.position(j+10); //PC_6  // KIRI DEPAN Z
	kid2.position(k-15); //PC_8 
	kid3.position(l-57);} //PA_11   +2

void kiri_belakang(double x, double y, double z){
	invers_3dof(x, y, z);

	if(teta1 >= 120){teta1 = 120;}
	else if(teta1 <= -120){teta1 = -120;}
	if(teta2 >= 130){teta2 = 130;}
	else if(teta2 <= -130){teta2 = -130;}

	kib1.position(teta3+16);
	kib2.position(teta1-6);
	kib3.position(teta2-74);}

void kanan_depan(double x, double y, double z){	
	invers_3dof(x+frontLegConst, y-legConstUukhai, z);	//frontLegConst=6; jalan biasa ::: frontLegConst=10: jalan mountain

	if(teta1 >= 120){teta1 = 120;}
	else if(teta1 <= -120){teta1 = -120;}
	if(teta2 >= 130){teta2 = 130;}
	else if(teta2 <= -130){teta2 = -130;}

	kad1.position(teta3-14);
 	kad2.position(teta1-2);
	kad3.position(teta2-60);}	//-39

void kanan_belakang(double x, double y, double z){		
	invers_3dof(x, y, z);

	if(teta1 >= 120){teta1 = 120;}
	else if(teta1 <= -120){teta1 = -120;}
	if(teta2 >= 130){teta2 = 130;}
	else if(teta2 <= -130){teta2 = -130;}
	if(teta3 >= 90){teta3 = 90;}
	else if(teta3 <= -90){teta3 = -90;}

	kab1.position(teta3+17);
	kab2.position(teta1+13);
	kab3.position(teta2-54);}

void kiri_depan(double x, double y, double z){
	invers_3dof(x+frontLegConst, y-legConstUukhai-0.5, z);		//frontLegConst=6; jalan biasa ::: frontLegConst=10: jalan mountain

	if(teta1 >= 120){teta1 = 120;}
	else if(teta1 <= -120){teta1 = -120;}
	if(teta2 >= 130){teta2 = 130;}
	else if(teta2 <= -130){teta2 = -130;}
	if(teta3 >= 90){teta3 = 90;}
	else if(teta3 <= -90){teta3 = -90;}
	
	kid1.position(teta3+12);
	kid2.position(teta1-15);
	kid3.position(teta2-63);}

void gripper(double x, double y){
	invers_gripper(x, y);
	
	if(teta1_2>=170){teta1_2=170;}
	else if(teta1_2<=-170){teta1_2=-170;}
	if(teta2_2>=170){teta2_2=170;}
	else if(teta2_2<=-170){teta2_2=-170;}
	if(teta3 >= 90){teta3 = 90;}
	else if(teta3 <= -90){teta3 = -90;}

	A.position(teta1_2-2);
	B.position(teta2_2+87);
	C.position(teta3_2+5);}

void atur_gripper(int a, int b, int c){
	A.position(a-5);
	B.position(b-80);//+87
	C.position(c+5);}

void gripper_open(){ //POSISI TANGAN BUKA
	atur_gripper(64,151,90);}

void gripper_half(){ //POSISI TANGAN BUKA
	atur_gripper(64,151,60);}

void gripper_close(){ //POSISI TANGAN GRIP
	atur_gripper(64,151,20);}

void duduk_anjing_manis2(){
	kiri_depan(-1, 10, 0);
	kanan_depan(0, 12.5, 0);
	kiri_belakang(6, 10, 0);
	kanan_belakang(6, 11, 0);}	
	
void duduk_anjing_manis(){
	kiri_depan(-1, 10, 0);
	kanan_depan(0, 12.5, 0);
	kiri_belakang(6, 10, 0);
	kanan_belakang(6, 11, 0);}

//===============================KINEMATICS INVERSE================================//
/*NOTE
	INVERS KINEMATIK KAKI*/

void invers(double x, double y, double z){
	kiri_depan(x, y, z);
	kanan_depan(x, y, z);
	kanan_belakang(x, y, z);
	kiri_belakang(x, y, z);}

void invers_gripper(double x, double y){	
	x = -1*x;		

	teta2_2 = acos((x*x+y*y-l1_grip*l1_grip-l2_grip*l2_grip)/(2*l1_grip*l2_grip));
	teta1_2 = atan2(y,x)-atan((l2_grip*sin(teta2_2))/(l1_grip+l2_grip*cos(teta2_2)));

	teta1_2 = -1*(90-teta1_2*(180/phi));
	teta2_2 = -1*(teta2_2*(180/phi));

	if(teta1_2>=0 && teta2_2<0){
		teta3_2=(teta2_2-teta1_2);}
	else if(teta1_2>=0 && teta2_2>=0){
		teta3_2=-1*(-teta2_2+teta1_2);}
	else if(teta1_2<0 && teta2_2>0){
		teta3_2=-1*(-teta1_2-teta2_2);}
	else if(teta1_2<0 && teta2_2<=0){
		teta3_2=-teta1_2+teta2_2;}}

void invers_2dof(double x, double y){
	x = -1*x;	
	teta2 = acos((x*x+y*y-l1*l1-l2*l2)/(2*l1*l2));
	teta1 = atan2(y,x)-atan((l2*sin(teta2))/(l1+l2*cos(teta2)));
	
	teta1 = 90-teta1*(180/phi);
	teta2 = teta2*(180/phi);}

void invers_3dof(double x, double y, double z){
	x = -1*x;		
	teta2 = acos((x*x+y*y-l1*l1-l2*l2)/(2*l1*l2));
	teta1 = atan2(y,x)-atan((l2*sin(teta2))/(l1+l2*cos(teta2)));
	
	teta3 = atan(z/y);	
	taw = sqrt(y*y+z*z)-l3;
	y = taw;
	
	teta1 = 90-teta1*(180/phi);
	teta2 = teta2*(180/phi);
	teta3 = teta3*(180/phi);}

//====================================================================CARA BERJALAN 1======================================================================//
/*NOTE	
		CARA BERJALAN 2 KAKI PER STEP:
		2 KAKI (KIRI DEPAN & KANAN BELAKANG) ANGKAT
			2 KAKI (KIRI BELAKANG & KANAN DEPAN) JALAN
	
	tiang		:: tinggi angkat setiap kaki
	tidiri	:: tinggi berdiri setiap kaki
	pasux		:: panjang sumbu x
	tiux		:: titik ujung terdepan sumbu x*/

void position(float tidiri, float tingah){
	invers(tingah, tidiri, 0);}

void sinus_pattern_start(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::
		FASE AWAL SINUS PATTERN, DARI DIAM
		KIRI DEPAN NAIK
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP*/
	
	pasux/=2;
	float tiux=tingah+pasux,
				varux=pasux-tiux, tingah2=tingah,
				timeSamp = phi/freqSamp, //NAIK
				timeSamp2=pasux/freqSamp; //MUNDUR
	
	for(float i=phi; i<=2*phi; i+=timeSamp){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;
		float x2=((i-phi)/phi)*pasux-varux;
		float x3=((i-phi)/phi)*pasux-varux;

		kiri_depan(x3, y, 0); //NAIK
		kanan_belakang(x2, y, 0); //NAIK

		kanan_depan(tingah2, tidiri, 0);
		kiri_belakang(tingah2, tidiri, 0);

		tingah2-=timeSamp2;
		wait_ms(1);}
	
	poster1=-tiux;
	poster2=tiux;
	poster3=tiux;
	poster4=-tiux;
	
	last_variabel_left=abs(tiux-tingah);
	last_variabel_right=abs(tiux-tingah);}

void sinus_pattern_fase1(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction){
	/*NOTE ::
		FASE 1 SINUS PATTERN
		KANAN DEPAN NAIK
		NAIK : KIBEL & KADEP
		MUNDUR : KABEL & KIDEP*/

	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,
				p1a, p1b, p1c, p1d,
				p2b, p2c;

	p1a=tiux-direction/2; //KADEP
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux+direction/2; //KIBEL

	p2b=tiux-pasux+direction/2; //KABEL
	p2c=tiux-pasux-direction/2; //KIDEP

	d1=abs(poster1-p1a);
	d2=abs(p2b-poster2);
	d3=abs(p2c-poster3);
	d4=abs(poster4-p1d);

	varux1=d1-p1a;
	varux4=d4-p1d;

	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y=tiang*sin(i)+tidiri;				

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4;

		kanan_depan(x1, y, 0); //NAIK
		kiri_belakang(x4, y, 0); //NAIK

		kanan_belakang(p1b, tidiri, 0);
		kiri_depan(p1c, tidiri, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}

	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;

	temporary_date=direction;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_pattern_fase2(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction){
	/*NOTE ::
		FASE 2 SINUS PATTERN
		KIRI DEPAN NAIK
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP*/

	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x2, x3,
				varux2, varux3, 
				p1a, p1b, p1c, p1d,
				p2a, p2d;
	
	p1a=poster1;	//KADEP
	p1b=tiux-direction/2; //KABEL
	p1c=tiux+direction/2; //KIDEP			
	p1d=poster4;	//KIBEL
	
	p2a=tiux-pasux+direction/2; //KADEP
	p2d=tiux-pasux-direction/2; //KIBEL				
		
	d1=abs(poster1-p2a);
	d2=abs(p1b-poster2);
	d3=abs(p1c-poster3);
	d4=abs(poster4-p2d);
	
	varux2=d2-p1b;
	varux3=d3-p1c;
	
	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL				
	
	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;		
		
		x2=((i-phi)/phi)*d2-varux2;
		x3=((i-phi)/phi)*d3-varux3;
		
		kanan_belakang(x2, y, 0); //NAIK
		kiri_depan(x3, y, 0); //NAIK
			
		kanan_depan(p1a, tidiri, 0);
		kiri_belakang(p1d, tidiri, 0);
				
		p1a-=ts1;
		p1d-=ts4;		
		wait_ms(1);}

	poster1=p2a;
	poster2=p1b;
	poster3=p1c;
	poster4=p2d;

	temporary_date=direction;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

	
	
	
void sinus_mundur_fase1(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction){
	/*NOTE ::
		FASE 1 SINUS PATTERN
		KANAN DEPAN NAIK
		NAIK : KIBEL & KADEP
		MUNDUR : KABEL & KIDEP*/

	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,
				p1a, p1b, p1c, p1d,
				p2b, p2c;

	p1a=tiux-direction/2; //KADEP
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux+direction/2; //KIBEL

	p2b=tiux-pasux+direction/2; //KABEL
	p2c=tiux-pasux-direction/2; //KIDEP

	d1=abs(poster1-p1a);
	d2=abs(p2b-poster2);
	d3=abs(p2c-poster3);
	d4=abs(poster4-p1d);

	varux1=d1-p1a;
	varux4=d4-p1d;

	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y=tiang*sin(i)+tidiri;				

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4;

		kanan_depan(-x1, y, 0); //NAIK
		kiri_belakang(-x4, y, 0); //NAIK

		kanan_belakang(-p1b, tidiri, 0);
		kiri_depan(-p1c, tidiri, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}

	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;

	temporary_date=direction;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_mundur_fase2(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction){
	/*NOTE ::
		FASE 2 SINUS PATTERN
		KIRI DEPAN NAIK
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP*/

	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x2, x3,
				varux2, varux3, 
				p1a, p1b, p1c, p1d,
				p2a, p2d;
	
	p1a=poster1;	//KADEP
	p1b=tiux-direction/2; //KABEL
	p1c=tiux+direction/2; //KIDEP			
	p1d=poster4;	//KIBEL
	
	p2a=tiux-pasux+direction/2; //KADEP
	p2d=tiux-pasux-direction/2; //KIBEL				
		
	d1=abs(poster1-p2a);
	d2=abs(p1b-poster2);
	d3=abs(p1c-poster3);
	d4=abs(poster4-p2d);
	
	varux2=d2-p1b;
	varux3=d3-p1c;
	
	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL				
	
	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;		
		
		x2=((i-phi)/phi)*d2-varux2;
		x3=((i-phi)/phi)*d3-varux3;
		
		kanan_belakang(-x2, y, 0); //NAIK
		kiri_depan(-x3, y, 0); //NAIK
			
		kanan_depan(-p1a, tidiri, 0);
		kiri_belakang(-p1d, tidiri, 0);
				
		p1a-=ts1;
		p1d-=ts4;		
		wait_ms(1);}

	poster1=p2a;
	poster2=p1b;
	poster3=p1c;
	poster4=p2d;

	temporary_date=direction;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}
	
	
	
	
	
	
	
	
//=======================================================================================================================
void sinus_pattern_stop1(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::
		KIRI DEPAN TUTUP 
		FASE 1 SINUS PATTERN
		NAIK : KIBEL & KADEP
		MUNDUR : KABEL & KIDEP*/
	
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x2, x3,
				varux2, varux3,
				p1a, p1b, p1c, p1d;
	
	p1a=poster1; //KADEP
	p1b=tiux;	//KABEL
	p1c=tiux;	//KIDEP
	p1d=poster4; //KIBEL

	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);

	varux2=d2-tingah;
	varux3=d3-tingah;

	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL				
	
	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;		
		
		x2=((i-phi)/phi)*d2-varux2;
		x3=((i-phi)/phi)*d3-varux3;
		
		kanan_belakang(x2, y, 0); //NAIK
		kiri_depan(x3, y, 0); //NAIK
			
		kanan_depan(p1a, tidiri, 0);
		kiri_belakang(p1d, tidiri, 0);											
				
		p1a-=ts1;
		p1d-=ts4;		
		wait_ms(1);}

	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;
	
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_pattern_stop2(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::
		KANAN DEPAN TUTUP
		FASE TUTUP SINUS PATTERN
		NAIK & TUTUP : KABEL & KIDEP
		MUNDUR & TUTUP : KIBEL & KADEP*/
	
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,				
				p1a, p1b, p1c, p1d;

	p1a=tiux; //KADEP	
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux; //KIBEL			
		
	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);
	
	varux1=d1-tingah;
	varux4=d4-tingah;
		
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y=tiang*sin(i)+tidiri;				

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4;

		kanan_depan(x1, y, 0); //NAIK
		kiri_belakang(x4, y, 0); //NAIK

		kanan_belakang(p1b, tidiri, 0);
		kiri_depan(p1c, tidiri, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}
	
	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;
		
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

	//=============================================================================================================================
void kanan1(float tiang, float tidiri, float pasux, float tingah, float tingah2, float freqSamp){
	/*NOTE ::
		KANAN DEPAN NAIK
		FASE 1 SINUS PATTERN
		NAIK : KIBEL & KADEP
		MUNDUR : KABEL & KIDEP*/
	pasux*=2;
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,				
				p1a, p1b, p1c, p1d,
				p2a, p2b, p2c, p2d;

	p1a=tiux; //KADEP	
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux; //KIBEL		

	p2a=tiux-pasux; //KADEP
	p2b=tiux-pasux; //KABEL
	p2c=tiux-pasux; //KIDEP
	p2d=tiux-pasux; //KIBEL		

	d1=abs(poster1-p1a);
	d2=abs(poster2-p2b);
	d3=abs(poster3-p2c);
	d4=abs(poster4-p1d);

	varux1=d1-p1a;	
	varux4=d4-p1d;
		
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL
	
	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y=tiang*sin(i)+tidiri;				

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4;
		
		kiri_belakang(tingah2, y, x4);
		kanan_belakang(tingah2, tidiri, p1b);
		
		kiri_depan(tingah2, tidiri, -p1b);
		kanan_depan(tingah2, y, -x4);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}

	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;
	
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void kanan2(float tiang, float tidiri, float pasux, float tingah, float tingah2, float freqSamp){
	/*NOTE ::
		KIRI DEPAN TUTUP
		FASE 2 SINUS PATTERN
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP*/
	pasux*=2;
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x2, x3,
				varux2, varux3, 
				p1a, p1b, p1c, p1d,
				p2a, p2d;
	
	p1a=poster1;	//KADEP
	p1b=tiux; //KABEL
	p1c=tiux; //KIDEP			
	p1d=poster4;	//KIBEL
		
	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);
	
	varux2=d2-tingah;
	varux3=d3-tingah;
	
	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;		

		x2=((i-phi)/phi)*d2-varux2;
		x3=((i-phi)/phi)*d3-varux3;

		kiri_depan(tingah2, y, -x2);
		kanan_depan(tingah2, tidiri, -p1d);
		
		kiri_belakang(tingah2, tidiri, p1d);
		kanan_belakang(tingah2, y, x2);

		p1a-=ts1;
		p1d-=ts4;		
		wait_ms(1);}

	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

//=======================================================================================================================
void kiri1(float tiang, float tidiri, float pasux, float tingah, float tingah2, float freqSamp){
	/*NOTE ::
		KANAN DEPAN NAIK
		FASE 1 SINUS PATTERN
		NAIK : KIBEL & KADEP
		MUNDUR : KABEL & KIDEP*/
	pasux*=2;
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,				
				p1a, p1b, p1c, p1d,
				p2a, p2b, p2c, p2d;

	p1a=tiux; //KADEP	
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux; //KIBEL		

	p2a=tiux-pasux; //KADEP
	p2b=tiux-pasux; //KABEL
	p2c=tiux-pasux; //KIDEP
	p2d=tiux-pasux; //KIBEL		

	d1=abs(poster1-p1a);
	d2=abs(poster2-p2b);
	d3=abs(poster3-p2c);
	d4=abs(poster4-p1d);

	varux1=d1-p1a;	
	varux4=d4-p1d;
		
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL
	
	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y=tiang*sin(i)+tidiri;				

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4;
		
		kiri_belakang(tingah2, tidiri, x4);
		kanan_belakang(tingah2, y, p1b);
		
		kiri_depan(tingah2, y, -p1b);
		kanan_depan(tingah2, tidiri, -x4);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}

	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;
	
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void kiri2(float tiang, float tidiri, float pasux, float tingah, float tingah2, float freqSamp){
	/*NOTE ::
		KIRI DEPAN TUTUP
		FASE 2 SINUS PATTERN
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP*/
	pasux*=2;			
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x2, x3,
				varux2, varux3, 
				p1a, p1b, p1c, p1d,
				p2a, p2d;
	
	p1a=poster1;	//KADEP
	p1b=tiux; //KABEL
	p1c=tiux; //KIDEP			
	p1d=poster4;	//KIBEL
		
	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);
	
	varux2=d2-tingah;
	varux3=d3-tingah;

	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y=tiang*sin(i)+tidiri;		

		x2=((i-phi)/phi)*d2-varux2;
		x3=((i-phi)/phi)*d3-varux3;

		kiri_depan(tingah2, tidiri, -x2);
		kanan_depan(tingah2, y, -p1d);
		
		kiri_belakang(tingah2, y, p1d);
		kanan_belakang(tingah2, tidiri, x2);

		p1a-=ts1;
		p1d-=ts4;		
		wait_ms(1);}

	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_sandune_fase1(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::		
		2 KAKI DEPAN NAIK SANDUNE
		KANAN DEPAN --> KIRI DEPAN*/

	float tiux=tingah+pasux/2, y_sandune=9,
				ts=abs((tingah-tiux))/freqSamp;

	for(float y=tidiri; y>=tidiri-tiang; y-=ts){ //KANAN DEPAN
		kanan_depan(tingah, y, 0);
		wait_ms(1);}	
	for(float x=tingah; x<=tiux; x+=ts){
		kanan_depan(x, tidiri-tiang, 0);
		wait_ms(1);}	
	for(float y=tidiri-tiang; y<=tidiri-y_sandune; y+=ts){
		kanan_depan(tiux, y, 0);
		wait_ms(1);}
	wait(0.1);
				
	for(float y=tidiri; y>=tidiri-tiang-5; y-=ts){ //KIRI DEPAN
		kiri_depan(tingah, y, 0);
		wait_ms(1);}	
	for(float x=tingah; x<=tiux; x+=ts){
		kiri_depan(x, tidiri-tiang-5, 0);
		wait_ms(1);}	
	for(float y=tidiri-tiang-5; y<=tidiri-y_sandune; y+=ts){
		kiri_depan(tiux, y, 0);
		wait_ms(1);}
	wait(0.1);}

void sinus_sandune_fase15(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tidepan){	
	/*NOTE ::
		FASE AWAL SINUS PATTERN, DARI DIAM
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP//*/		
	
	pasux/=2;
	float tiux=tingah+pasux,
				varux=pasux-tiux, tingah2=tingah,
				timeSamp = phi/freqSamp, //NAIK
				timeSamp2=pasux/freqSamp; //MUNDUR
	
	for(float i=phi; i<=2*phi; i+=timeSamp){ //KIDEP & KABEL NAIK		
		float y2=tiang*sin(i)+tidiri;
		float y3=tiang*sin(i)+tidepan;
		
		float x2=((i-phi)/phi)*pasux-varux-5;
		float x3=((i-phi)/phi)*pasux-varux;
		
		kanan_depan(tingah2, tidepan, 0);
		kiri_belakang(tingah2, tidiri, 0);
		
		kanan_belakang(x2, y2, 0); //NAIK
		kiri_depan(x3, y3, 0); //NAIK

		tingah2-=timeSamp2;						
		wait_ms(1);}
	
	poster1=tingah-pasux;
	poster2=tingah+pasux;
	poster3=tingah+pasux;
	poster4=tingah-pasux;
	
	last_variabel_left=abs(tiux-tingah);
	last_variabel_right=abs(tiux-tingah);} 

void sinus_sandune_fase2(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tidepan){
	/*NOTE ::
		JALAN 1 SIKLUS DENGAN KAKI DEPAN DI ATAS SANDUNE, KAKI BELAKANG DI BAWAH
		-KAKI DEPAN LANGKAH KECIL*/

	float ts1, ts2, ts3, ts4, tiux=tingah+pasux/2,

				p1a=tiux, p2a=tiux-pasux, //KADEP
				p1b=tiux, p2b=tiux-pasux, //KABEL 
				p1c=tiux, p2c=tiux-pasux, //KIDEP
				p1d=tiux, p2d=tiux-pasux, //KIBEL

				d1=abs(p1a-p2a), d2=abs(p1b-p2b),
				d3=abs(p1c-p2c), d4=abs(p1d-p2d),

				varux1=d1-p1a, varux2=d2-p1b, 
				varux3=d3-p1c, varux4=d4-p1d;
	
	//=======================FASE 1======================	
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL	

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK				
		float y1=tiang*sin(i)+tidepan;
		float y4=tiang*sin(i)+tidiri;

		float x1=((i-phi)/phi)*pasux-varux1;
		float x4=((i-phi)/phi)*pasux-varux4-5;

		kanan_depan(x1, y1, 0);
		kiri_belakang(x4, y4, 0);

		kanan_belakang(p1b-5, tidiri, 0);
		kiri_depan(p1c, tidepan, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}
	
	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;
		
	//=======================FASE 2======================	
	p1a=poster1;	//KADEP
	p1b=tiux; //KABEL
	p1c=tiux; //KIDEP			
	p1d=poster4;	//KIBEL
	
	p2a=tiux-pasux; //KADEP
	p2d=tiux-pasux; //KIBEL				
		
	d1=abs(poster1-p2a);
	d2=abs(p1b-poster2);
	d3=abs(p1c-poster3);
	d4=abs(poster4-p2d);
	
	varux2=d2-p1b;
	varux3=d3-p1c;

	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL	
		
	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y2=tiang*sin(i)+tidiri;	
		float y3=tiang*sin(i)+tidepan;		

		float x2=((i-phi)/phi)*pasux-varux2-5;
		float x3=((i-phi)/phi)*pasux-varux3;		
		
		kanan_belakang(x2, y2, 0);
		kiri_depan(x3, y3, 0);
			
		kanan_depan(p1a, tidepan, 0);
		kiri_belakang(p1d-5, tidiri, 0);
		
		p1a-=ts1;
		p1d-=ts4;				
		wait_ms(1);}

	poster1=p2a;
	poster2=p1b;
	poster3=p1c;
	poster4=p2d;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_sandune_fase25(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tidepan){
	/*NOTE ::
		KANAN DEPAN TUTUP
		JALAN 1 SIKLUS DENGAN KAKI DEPAN DI ATAS SANDUNE, KAKI BELAKANG DI BAWAH
		NAIK & TUTUP : KABEL & KIDEP
		MUNDUR & TUTUP : KIBEL & KADEP*/
	
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,				
				p1a, p1b, p1c, p1d;

	p1a=tiux; //KADEP	
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux; //KIBEL			
		
	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);
	
	varux1=d1-tingah;
	varux4=d4-tingah;
		
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y1=tiang*sin(i)+tidepan;
		float y4=tiang*sin(i)+tidiri;

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4-5;

		kanan_depan(x1, y1, 0); //NAIK
		kiri_belakang(x4, y4, 0); //NAIK

		kanan_belakang(p1b-5, tidiri, 0);
		kiri_depan(p1c, tidepan, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}
	
	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;
		
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_sandune_fase3(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::		
		2 KAKI BELAKANG NAIK SANDUNE
		KANAN BELAKANG --> KIRI BELAKANG*/
	
	float tiux=tingah+pasux/2, y_sandune=5,
				ts=abs((tingah-tiux))/freqSamp;

	for(float y=22; y>=20; y-=0.02){
		kiri_depan(0, y, 0);
		kanan_depan(0, y, 0);
		wait_ms(1);}
	wait(0.5
		);
		
	for(float y=0; y<=7; y+=ts){ //KIRI BELAKANG
		kiri_belakang(tingah-y, tidiri-y, 0);
		wait_ms(1);}
	wait(0.05);
	for(float y=tidiri-7; y>=tidiri-tiang; y-=ts){
		kiri_belakang(tingah-7, y, 0);
		wait_ms(1);}	
	wait(0.05);
	for(float x=tingah; x<=tiux; x+=ts){
		kiri_belakang(x, tidiri-tiang, 0);
		wait_ms(1);}	
	wait(0.05);
	for(float y=tidiri-tiang; y<=tidiri-y_sandune; y+=ts){
		kiri_belakang(tiux, y, 0);
		wait_ms(1);}
	wait(0.05);

	for(float x=tingah; x>=tingah-7; x-=ts){ //KANAN BELAKANG
		kanan_belakang(x, tidiri, 0);
		wait_ms(1);}
	wait(0.05);
	for(float y=tidiri; y>=tidiri-tiang; y-=ts){ 
		kanan_belakang(tingah-7, y, 0);
		wait_ms(1);}
	wait(0.05);
	for(float x=tingah-7; x<=tiux; x+=ts){
		kanan_belakang(x, tidiri-tiang, 0);
		wait_ms(1);}	
	wait(0.05);
	for(float y=tidiri-tiang; y<=tidiri-y_sandune; y+=ts){
		kanan_belakang(tiux, y, 0);
		wait_ms(1);}}

void sinus_sandune_fase35(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tibelakang){
	/*NOTE ::
		FASE AWAL SINUS PATTERN, DARI DIAM
		NAIK : KABEL & KIDEP
		MUNDUR : KIBEL & KADEP//*/		
	
	pasux/=2;
	float tiux=tingah+pasux,
				varux=pasux-tiux, tingah2=tingah,
				timeSamp = phi/freqSamp, //NAIK
				timeSamp2=pasux/freqSamp; //MUNDUR
	
	for(float i=phi; i<=2*phi; i+=timeSamp){ //KIDEP & KABEL NAIK		
		float y2=tiang*sin(i)+tibelakang;
		float y3=tiang*sin(i)+tidiri;
		
		float x2=((i-phi)/phi)*pasux-varux-5;
		float x3=((i-phi)/phi)*pasux-varux;
		
		kanan_depan(tingah2, tidiri, 0);
		kiri_belakang(tingah2, tibelakang, 0);
		
		kanan_belakang(x2, y2, 0); //NAIK
		kiri_depan(x3, y3, 0); //NAIK

		tingah2-=timeSamp2;						
		wait_ms(1);}
	
	poster1=tingah-pasux;
	poster2=tingah+pasux;
	poster3=tingah+pasux;
	poster4=tingah-pasux;
	
	last_variabel_left=abs(tiux-tingah);
	last_variabel_right=abs(tiux-tingah);} 

void sinus_sandune_fase4(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tibelakang){
	/*NOTE ::
		-JALAN DENGAN KAKI DEPAN DI BAWAH KAKI BELAKANG DI SANDUNE
		-KAKI DEPAN LANGKAH KECIL*/	

	float ts1, ts2, ts3, ts4,
				tiux=tingah+pasux/2,

				p1a=tiux, p2a=tiux-pasux, //KADEP
				p1b=tiux, p2b=tiux-pasux, //KABEL 
				p1c=tiux, p2c=tiux-pasux, //KIDEP
				p1d=tiux, p2d=tiux-pasux, //KIBEL

				d1=abs(p1a-p2a), d2=abs(p1b-p2b),
				d3=abs(p1c-p2c), d4=abs(p1d-p2d),

				varux1=d1-p1a, varux2=d2-p1b, 
				varux3=d3-p1c, varux4=d4-p1d;
	
	//=======================FASE 1======================	
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL		

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK				
		float y1=tiang*sin(i)+tidiri;
		float y4=tiang*sin(i)+tibelakang;

		float x1=((i-phi)/phi)*pasux-varux1;
		float x4=((i-phi)/phi)*pasux-varux4;

		kanan_depan(x1, y1, 0); //NAIK
		kiri_belakang(x4, y4, 0); //NAIK

		kanan_belakang(p1b, tibelakang, 0);
		kiri_depan(p1c, tidiri, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}
	
	poster1=p1a;
	poster2=p2b;
	poster3=p2c;
	poster4=p1d;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;
		
	//=======================FASE 2======================	
	p1a=poster1;	//KADEP
	p1b=tiux; //KABEL
	p1c=tiux; //KIDEP			
	p1d=poster4;	//KIBEL
	
	p2a=tiux-pasux; //KADEP
	p2d=tiux-pasux; //KIBEL
	
	varux2=d2-p1b;
	varux3=d3-p1c;

	ts1=d1/freqSamp; //KADEP
	ts2=phi/freqSamp; //KABEL
	ts3=phi/freqSamp; //KIDEP
	ts4=d4/freqSamp; //KIBEL	
		
	for(float i=phi; i<=2*phi; i+=ts2){ //KIDEP & KABEL NAIK		
		float y2=tiang*sin(i)+tibelakang;
		float y3=tiang*sin(i)+tidiri;

		float x2=((i-phi)/phi)*pasux-varux2;
		float x3=((i-phi)/phi)*pasux-varux3;		
		
		kanan_belakang(x2, y2, 0);
		kiri_depan(x3, y3, 0);

		kanan_depan(p1a, tidiri, 0);
		kiri_belakang(p1d, tibelakang, 0);
		
		p1a-=ts1;
		p1d-=ts4;
		wait_ms(1);}

	poster1=p2a;
	poster2=p1b;
	poster3=p1c;
	poster4=p2d;

	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void sinus_sandune_fase45(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tibelakang){
	/*NOTE ::
		KANAN DEPAN TUTUP
		JALAN 1 SIKLUS DENGAN KAKI DEPAN DI ATAS SANDUNE, KAKI BELAKANG DI BAWAH
		NAIK & TUTUP : KABEL & KIDEP
		MUNDUR & TUTUP : KIBEL & KADEP*/
	
	float tiux=tingah+pasux/2,
				ts1, ts2, ts3, ts4,
				d1, d2, d3, d4, x1, x4,
				varux1, varux4,				
				p1a, p1b, p1c, p1d;

	p1a=tiux; //KADEP	
	p1b=poster2;	//KABEL
	p1c=poster3;	//KIDEP
	p1d=tiux; //KIBEL			
		
	d1=abs(poster1-tingah);
	d2=abs(poster2-tingah);
	d3=abs(poster3-tingah);
	d4=abs(poster4-tingah);
	
	varux1=d1-tingah;
	varux4=d4-tingah;
		
	ts1=phi/freqSamp; //KADEP
	ts2=d2/freqSamp; //KABEL
	ts3=d3/freqSamp; //KIDEP
	ts4=phi/freqSamp; //KIBEL				

	for(float i=phi; i<=2*phi; i+=ts1){ //KADEP & KIBEL NAIK
		float y1=tiang*sin(i)+tidiri;
		float y4=tiang*sin(i)+tibelakang;

		x1=((i-phi)/phi)*d1-varux1;
		x4=((i-phi)/phi)*d4-varux4-5;

		kanan_depan(x1, y1, 0); //NAIK
		kiri_belakang(x4, y4, 0); //NAIK

		kanan_belakang(p1b-5, tibelakang, 0);
		kiri_depan(p1c, tidiri, 0);

		p1b-=ts2;
		p1c-=ts3;
		wait_ms(1);}
	
	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;
		
	temporary_date=0;
	last_variabel_left=d4/2;
	last_variabel_right=d1/2;}

void tali1(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::		
		2 KAKI DEPAN NAIK SANDUNE
		KANAN DEPAN --> KIRI DEPAN*/	
	float tiux=tingah+pasux/2, y_sandune=9,
				ts=abs((tingah-tiux))/freqSamp;

	for(float y=0; y<=12; y+=ts){ //KANAN DEPAN
		kanan_depan(tingah-y, tidiri-y, 0);
		wait_ms(1);}
	for(float y=tidiri-12; y>=tidiri-tiang; y-=ts){
		kanan_depan(tingah-12, y, 0);
		wait_ms(1);}	
	for(float x=tingah-12; x<=tiux; x+=ts){
		kanan_depan(x, tidiri-tiang, 0);
		wait_ms(1);}		
	for(float y=tidiri-tiang; y<=tidiri-y_sandune; y+=ts){
		kanan_depan(tiux, y, 0);
		wait_ms(1);}
	for(float x=tiux; x>=tingah; x-=ts){
		kanan_depan(x, tidiri, 0);
		wait_ms(1);}
	wait(0.3);

	for(float y=0; y<=12; y+=ts){ //KIRI DEPAN
		kiri_depan(tingah-y, tidiri-y, 0);
		wait_ms(1);}
	for(float y=tidiri-12; y>=tidiri-tiang; y-=ts){
		kiri_depan(tingah-12, y, 0);
		wait_ms(1);}	
	for(float x=tingah-12; x<=tiux; x+=ts){
		kiri_depan(x, tidiri-tiang, 0);
		wait_ms(1);}		
	for(float y=tidiri-tiang; y<=tidiri-y_sandune; y+=ts){
		kiri_depan(tiux, y, 0);
		wait_ms(1);}
	for(float x=tiux; x>=tingah; x-=ts){
		kiri_depan(x, tidiri, 0);
		wait_ms(1);}
	wait(0.3);}

void tali2(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	/*NOTE ::		
		2 KAKI DEPAN NAIK SANDUNE
		KANAN DEPAN --> KIRI DEPAN*/

	float tiux=tingah+pasux/2, y_sandune=9,
				ts=abs((tingah-tiux))/freqSamp;

	for(float y=0; y<=12; y+=ts){ //KANAN BELAKANG
		kanan_belakang(tingah-y, tidiri-y, 0);
		wait_ms(1);}
	for(float y=tidiri-12; y>=tidiri-tiang; y-=ts){
		kanan_belakang(tingah-12, y, 0);
		wait_ms(1);}	
	for(float x=tingah-12; x<=tiux; x+=ts){
		kanan_belakang(x, tidiri-tiang, 0);
		wait_ms(1);}		
	for(float y=tidiri-tiang; y<=tidiri-y_sandune+3; y+=ts){
		kanan_belakang(tiux, y, 0);
		wait_ms(1);}
	for(float x=tiux; x>=tingah; x-=ts){
		kanan_belakang(x, tidiri-y_sandune+3, 0);
		wait_ms(1);}
	for(float y=tidiri-y_sandune+3; y<=tidiri; y+=ts){
		kanan_belakang(tingah, y, 0);
		wait_ms(1);}
	wait(0.5);

	for(float y=0; y<=12; y+=ts){ //KIRI BELAKANG
		kiri_belakang(tingah-y, tidiri-y, 0);
		wait_ms(1);}
	for(float y=tidiri-12; y>=tidiri-tiang; y-=ts){
		kiri_belakang(tingah-12, y, 0);
		wait_ms(1);}	
	for(float x=tingah-12; x<=tiux; x+=ts){
		kiri_belakang(x, tidiri-tiang, 0);
		wait_ms(1);}		
	for(float y=tidiri-tiang; y<=tidiri-y_sandune+3; y+=ts){
		kiri_belakang(tiux, y, 0);
		wait_ms(1);}
	for(float x=tiux; x>=tingah; x-=ts){
		kiri_belakang(x, tidiri-y_sandune+3, 0);
		wait_ms(1);}
	for(float y=tidiri-y_sandune+3; y<=tidiri; y+=ts){
		kiri_belakang(tingah, y, 0);
		wait_ms(1);}
	wait(0.5);}

//==========================================================SENSORS INTERFACE AND FILTER PROGRAM==========================================================//
void offsetMagnetometer(){
	tempX = mx;
	tempY = my;
	tempZ = mz;
	
	tempX -= (tempMinX + tempMaksX)/2;
	tempY -= (tempMinY + tempMaksY)/2;
	tempZ -= (tempMinZ + tempMaksZ)/2;}

void compassMin(float miX, float miY, float miZ){
	tempMinX=miX; 
	tempMinY=miY;
	tempMinZ=miZ;}

void compassMax(float maX, float maY, float maZ){
	tempMaksX=maX; 
	tempMaksY=maY;
	tempMaksZ=maZ;}
	
void vectorCross1(float aX, float aY, float aZ, float bX, float bY, float bZ){
	Ex = aY*bZ - aZ*bY;
	Ey = aZ*bX - aX*bZ;
	Ez = aX*bY - aY*bX;}

void vectorNormalize1(float a, float b, float c){
	vectorDot1A(Ex, Ey, Ez);
	float mag = sqrt(vectorDotResult1A);
	Ex = a/mag;
	Ey = b/mag;
	Ez = c/mag;}

void vectorCross2(float aX, float aY, float aZ, float bX, float bY, float bZ){
	Nx = aY*bZ - aZ*bY;
	Ny = aZ*bX - aX*bZ;
	Nz = aX*bY - aY*bX;}

void vectorNormalize2(float a, float b, float c){
	vectorDot2A(Nx, Ny, Nz);
	float mag = sqrt(vectorDotResult2A);
	Nx = a/mag;
	Ny = b/mag;
	Nz = c/mag;}

void vectorDot1(float A, float B, float C){
	vectorDotResult1 = A*0 + B*-1 + C*0;}

void vectorDot1A(float A, float B, float C){
	vectorDotResult1A = A*A + B*B + C*C;}

void vectorDot2(float A, float B, float C){
	vectorDotResult2 = A*0 + B*-1 + C*0;}

void vectorDot2A(float A, float B, float C){
	vectorDotResult2A = A*A + B*B + C*C;}

void filterMPU9250(){		//VEKTOR FILTER AND COMPLEMENTARY FILTER FOR MPU 9250
	offsetMagnetometer();
	
	vectorCross1(tempX, tempY, tempZ, ax, ay, az);
	vectorNormalize1(Ex, Ey, Ez);	
	vectorCross2(ax, ay, az, Ex, Ey, Ez);
	vectorNormalize2(Nx, Ny, Nz);
	
	vectorDot1(Ex, Ey, Ez);
	vectorDot2(Nx, Ny, Nz);
	heading = atan2(vectorDotResult1, vectorDotResult2)*degConvert;
	heading -= constKalibrasi;
	if(heading >= 0){
		heading=heading;}
	else{
		heading=heading+360;}

	pitch=atan2(ax, az)*degConvert;		//PITCH ORIENTATION	:: FROM ACCEL, GYRO
	roll=atan2(ay, az)*degConvert;		//ROLL ORIENTATION	:: FROM ACCEL, GYRO
	yaw=heading;											//YAW ORIENTATION 	:: FROM ACCEL, GYRO, MAGNETO
		
	gyroXrate = gx/131;
	gyroYrate = gy/131;
	gyroZrate = gz/131;

	compAngleX = 0.9*(compAngleX+gyroXrate*dt)+0.1*roll;
	compAngleY = 0.9*(compAngleY+gyroYrate*dt)+0.1*pitch;
	compAngleZ = 0.9*(compAngleZ+gyroZrate*dt)+0.1*yaw;

	averageFilter(compAngleZ, 250);}

void readMPU9250(){
	mpu9250.readAccelData(accelCount); //Calculate the accleration value into actual g's
	ax = (float)accelCount[0]*aRes - accelBias[0];
	ay = (float)accelCount[1]*aRes - accelBias[1];
	az = (float)accelCount[2]*aRes - accelBias[2];

	mpu9250.readGyroData(gyroCount);  //Calculate the gyro value into actual degrees per second
	gx = ((float)gyroCount[0]*gRes - gyroBias[0]);
	gy = ((float)gyroCount[1]*gRes - gyroBias[1]);
	gz = ((float)gyroCount[2]*gRes - gyroBias[2]);

	mpu9250.readMagData(magCount);  //Calculate the magnetometer values in milliGauss
	mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];
	my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
	mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];}

void averageFilter(int a, int jmlh_ulang){
	jumlah=0;
	filterCompass[jmlh_ulang]=a;
	for(int count=0; count<=jmlh_ulang-1; count++){
		filterCompass[count]=filterCompass[count+1];
		jumlah=jumlah+filterCompass[count];}
	COMPASS=jumlah/jmlh_ulang;}

void mpu9250_init(){
	i2c.frequency(400000);	//USE FAST (400KHZ) I2C
	uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  //Read WHO_AM_I register for MPU-9250

	if(whoami == 0x71){		//WHO_AM_I should always be 0x68
		stmpc.printf("MPU9250 is online...\n\r");
		
		mpu9250.resetMPU9250(); //Reset registers to default in preparation for device calibration
		mpu9250.calibrateMPU9250(gyroBias, accelBias); //Calibrate gyro and accelerometers, load biases in bias registers

		mpu9250.initMPU9250(); 
		mpu9250.initAK8963(magCalibration);
		//break;
	}

	else{	//Loop forever if communication doesn't happen
		stmpc.printf("Could not connect to MPU9250: \n\r");
		stmpc.printf("%#x \n",  whoami);
		//while(1);
	}

	mpu9250.getAres(); // Get accelerometer sensitivity
	mpu9250.getGres(); // Get gyro sensitivity
	mpu9250.getMres(); // Get magnetometer sensitivity
	magbias[0] = +470.;	//User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = +120.;  //User environmental y-axis correction in milliGauss
	magbias[2] = +125.;  //User environmental z-axis correction in milliGauss
}

void MPU9250(){
	readMPU9250();
	filterMPU9250();}

void tembak_ultrasonik(){
	variabel_ultra=0;
	variabel_jumlah=0;	

	trigPin=0;
	wait_ms(2);

	trigPin=1;
	wait_us(10);

	trigPin=0;		
	while(echoPin==0){}
	while(echoPin==1 && variabel_ultra<=9960){
		variabel_ultra++;}				
	jarak_bawah=variabel_ultra/200;}

void ultrasonicFilter_init(){
	for(int i=0; i<=5; i++){
		filterUltrasonik[i]=20;}}

void rotasiKe(int SP){
	while(1){
		for(int i=0; i<=500; i++){
			MPU9250();}
		float eror = COMPASS - SP;
		stmpc.printf("%f, %f\n\r", COMPASS, eror);

		if(eror>=10){
			rotasi_kanan(5, 1);}
		else if(eror>1.4 && eror<10){
			rotasi_kanan(konstanta_belok*(eror), 1);}
		else if(eror<=-10){
			rotasi_kiri(5, 1);}
		else if(eror>-10 && eror<-1.4){
			rotasi_kiri(konstanta_belok*(-eror), 1);}
		else if(eror>=-1.4 && eror<=1.4){
			/*if(mode[0]=='R' && counting==2){					//RED :: MERAH 1 OTW SANDUNE
				counting=3;
				break;}
			else if(mode[0]=='B' && counting==2){			//BLUE :: BIRU 1 OTW SANDUNE
				break;}
			else if(mode[0]=='R' && counting==4){			//RED :: MERAH 1 OTW SANDUNE
				counting=5;
				break;}
			else if(mode[0]=='B' && counting==4){			//BLUE :: BIRU 1 OTW SANDUNE
				break;}*/
			break;}}
	stmpc.printf("LEWAT\n\r");}

void bacaSensordoang(){
	MPU9250();
	tembak_ultrasonik();
	stmpc.printf("%.2f || %.2f || %.2f || %.2f\n\r", COMPASS, compAngleX, compAngleY, jarak_bawah);}
//==========================================================OTHERS IMPORTANT PROGRAM==========================================================//
void start_position(float tidiri, float tingah){
	position(tidiri, tingah);
	
	poster1=tingah;
	poster2=tingah;
	poster3=tingah;
	poster4=tingah;}
		
void start(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	sinus_pattern_start(tiang, tidiri, pasux, tingah, freqSamp);}

void jalan(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction, float jml_ulang){
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_pattern_fase1(tiang, tidiri, pasux, tingah, freqSamp, direction);
		sinus_pattern_fase2(tiang, tidiri, pasux, tingah, freqSamp, direction);}}

void jalan_uukhai(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction, float jml_ulang){
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_pattern_fase1(tiang, tidiri, pasux, tingah, freqSamp, direction);
		sinus_pattern_fase2(tiang, tidiri, pasux, tingah, freqSamp, direction);
		wait(0.5);}}

void jalan1(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction, float jml_ulang){
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_pattern_fase1(tiang, tidiri, pasux, tingah, freqSamp, direction);}}

void jalan2(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction, float jml_ulang){
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_pattern_fase2(tiang, tidiri, pasux, tingah, freqSamp, direction);}}

void jalan_withimage(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float direction){
	date_received();
	sinus_pattern_fase1(tiang, tidiri, pasux, tingah, freqSamp, direction);
	date_received();
	sinus_pattern_fase2(tiang, tidiri, pasux, tingah, freqSamp, direction);}

void stop1(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	sinus_pattern_stop1(tiang, tidiri, pasux, tingah, freqSamp);}
	
void stop2(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	sinus_pattern_stop2(tiang, tidiri, pasux, tingah, freqSamp);}

void angkat_depan(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	sinus_sandune_fase1(tiang, tidiri, pasux, tingah, freqSamp);}

void jalan_depan_naik(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tidepan, float jml_ulang){
	sinus_sandune_fase15(tiang, tidiri, pasux, tingah, freqSamp, tidepan);
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_sandune_fase2(tiang, tidiri, pasux, tingah, freqSamp, tidepan);}
	sinus_sandune_fase25(tiang, tidiri, pasux, tingah, freqSamp, tidepan);}

void angkat_belakang(float tiang, float tidiri, float pasux, float tingah, float freqSamp){
	sinus_sandune_fase3(tiang, tidiri, pasux, tingah, freqSamp);}

void jalan_belakang_naik(float tiang, float tidiri, float pasux, float tingah, float freqSamp, float tibelakang, float jml_ulang){
	//sinus_sandune_fase35(tiang, tidiri, pasux, tingah, freqSamp, tibelakang);
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		sinus_sandune_fase4(tiang, tidiri, pasux, tingah, freqSamp, tibelakang);}
	sinus_sandune_fase45(tiang, tidiri, pasux, tingah, freqSamp, tibelakang);
}

void rotasi_kiri(float panjang, float jml_ulang){
	poster1=0;
	poster2=0;
	poster3=0;
	poster4=0;
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		kiri1(6, 24, panjang, 0, -6, 250);
		wait(0.1);
		kiri2(6, 24, panjang, 0, -6, 250);
		wait(0.5);}}

void rotasi_kanan(float panjang, float jml_ulang){
	poster1=0;
	poster2=0;
	poster3=0;
	poster4=0;
	for(int ulang=1; ulang<=jml_ulang; ulang++){
		kanan1(6, 24, panjang, 0, -6, 250);
		wait(0.1);
		kanan2(6, 24, panjang, 0, -6, 250);
		wait(0.5);}}

void date_received(){
	length=0;
	verified=0;	
	/*while(length == verified){
		if(stmrasp.readable()){
			data[length]=stmrasp.getc();
			if(data[length]!='.'){
				length++;
				verified++;}
			else{
				data[length]=' ';
				line_position=(atoi(data)-100)/20;
				length=0;}}}
	if(line_position>5){
		line_position=5;}
	else if(line_position<-5){
		line_position=-5;}*/}

void servo_init(){
	kad1.calibrate(0.001,135);	//Z
	kad2.calibrate(0.001,135);
	kad3.calibrate(0.001,90);

	kid1.calibrate(0.001,135); 	//Z
	kid2.calibrate(0.001,135);
	kid3.calibrate(0.001,90);

	kab1.calibrate(0.001,135);	//Z
	kab2.calibrate(0.001,135);
	kab3.calibrate(0.001,90);

	kib1.calibrate(0.001,135);	//Z
	kib2.calibrate(0.001,135);
	kib3.calibrate(0.001,90);

	A.calibrate(0.001,90);			//Z
	B.calibrate(0.001,90);
	C.calibrate(0.001,90);}

int whistle(){
	whistleValue = peluit.read();
	if(whistleValueLast == 1 && whistleValue == 0){
		value = 1;}
	else{
		value = 0;}
	whistleValueLast = whistleValue;	
	return value;}

void duduk_anjingku(){
	for(float i=24; i>=10; i=i-0.1){
		start_position(i, -9); //y,x
		wait(0.01);}
	for(float i=-9; i<=-1; i=i+0.1){
		kanan_depan		(i, 10, 0); //x,y,z
		kiri_depan		(i, 10, 0);
		kanan_belakang(i+7, 10, 0);
		kiri_belakang	(i+7, 10, 0);
		wait(0.01);}}

void berdiri_anjingku(){
	for(float i=-1; i>=-9; i=i-0.1){
		kanan_depan		(i, 10, 0); //x,y,z
		kiri_depan		(i, 10, 0);
		kanan_belakang(i+7, 10, 0);
		kiri_belakang	(i+7, 10, 0);
		wait(0.01);}
	for(float i=10; i<=24; i=i+0.1){
		start_position(i, -9); //y,x
		wait(0.01);}}

void salamKePenonton(){
	for(float i=-9; i<=-5; i+=0.075){
		start_position(24, i);
		wait(0.01);}
	wait(0.5);
	
	for(int i=0; i<=1; i++){
		for(float i=0; i<=4; i+=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		for(float i=4; i>=0; i-=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		wait(0.2);}
	wait(0.5);
	
	rotasi_kanan(8, 1);
	for(int i=0; i<=1; i++){
		for(float i=0; i<=4; i+=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		for(float i=4; i>=0; i-=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		wait(0.2);}
	wait(0.5);
	
	rotasi_kiri(8, 2);
	for(int i=0; i<=1; i++){
		for(float i=0; i<=4; i+=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		for(float i=4; i>=0; i-=0.075){
			kanan_depan		(-5, 24-i, 0); //x,y,z
			kiri_depan		(-5, 24-i, 0);
			kanan_belakang(-5, 24+i, 0);
			kiri_belakang	(-5, 24+i, 0);
			wait(0.01);}
		wait(0.2);}
	
	for(float i=-5; i>=-9; i-=0.075){
		start_position(24, i);
		wait(0.01);}
	wait(0.5);}

void salamDenganOrang(){
	bool humanAvailable, infraRed, infraRedLast, ACUAN;
	for(float y=24; y<=26; y+=0.01){
		invers(-9, y, 0);
		wait_ms(1);}
	wait(0.5);
	for(float x=-9; x<=-3; x+=0.01){
		invers(x, 26, 0);
		wait_ms(1);}
	wait(0.5);
		
	while(!humanAvailable){
		infraRed = infrared_uukhai.read();
		
		if(!infraRed && ACUAN == 0){ 	//ada orang dan angkat tangan
			for(float y=26; y>=10; y-=0.025){ 
				kanan_depan(-3, y, 0);
				wait_ms(1);}
			for(float x=-3; x<=8; x+=0.025){ 
				kanan_depan(x, 10, 0);
				wait_ms(1);}
			ACUAN = 1;}
		
		if(!infraRed && ACUAN == 1){ //ada orang dan goyang tangan
			for(float y=10; y>=5; y-=0.025){ 
				kanan_depan(8, y, 0);
				wait_ms(1);}
			for(float y=5; y<=15; y+=0.025){ 
				kanan_depan(8, y, 0);
				wait_ms(1);}
			for(float y=15; y>=10; y-=0.025){ 
				kanan_depan(8, y, 0);
				wait_ms(1);}}
		
		if(infraRed == 1 && infraRedLast == 0 && ACUAN == 1){	//orangnya sudah pergi
			for(float x=8; x>=-3; x-=0.025){ 
				kanan_depan(x, 10, 0);
				wait_ms(1);}
			for(float y=10; y<=26; y+=0.025){ 
				kanan_depan(-3, y, 0);
				wait_ms(1);}
			humanAvailable = 1;
			break;}
		
		infraRedLast = infraRed;}
	
	for(float x=-3; x>=-9; x-=0.01){
		invers(x, 26, 0);
		wait_ms(1);}
	wait(0.5);
	for(float y=26; y>=24; y-=0.01){
		invers(-9, y, 0);
		wait_ms(1);}
	wait(0.5);}

void jalanDiTempat(){
	start(7, 24, 0, -9, 240);
	jalan(7, 24, 0, -9, 240, 0, 10);
	stop2(7, 24, 0, -9, 240);}

void goyangEkor(){
	for(float i=0; i<=phi/4; i+=0.003){
		float nilai = 90*sin(i);
		A.position(nilai);
		wait_ms(1);}
	wait(0.75);
	for(float i=phi/4; i>=0; i-=0.003){
		float nilai = 90*sin(i);
		A.position(nilai);
		wait_ms(1);}
	wait(0.75);}

//===============================MAIN PROGRAM================================//
int main(){
	ultrasonicFilter_init();
	servo_init();

	tulis(0,0,0,
				0,0,0,
				0,0,0,
				0,0,0);

	start_position(24, -9);
	//gripper_half();
	//mpu9250_init();
	//gripper_open();
	stmpc.printf("GORDON ASHIAPPPP!\n\r");
	while(1){
		nilaiPeluit = whistle();
		if(nilaiPeluit == 1){
	
			if(STEP == 0){
				wait(0.5);
				duduk_anjingku();
				STEP-=-1;
				wait(2.5);}
			else if(STEP == 1){
				wait(0.5);
				berdiri_anjingku();
				STEP-=-1;
				wait(2.5);}
			else if(STEP == 2){
				wait(0.5);
				salamDenganOrang();
				STEP-=-1;
				wait(2.5);}
			else if(STEP == 3){
				wait(0.5);
				jalanDiTempat();
				STEP-=-1;
				wait(2.5);}
			else if(STEP == 4){
				wait(0.5);
				salamKePenonton();
				STEP-=-1;
				wait(2.5);}

		}
		else{
			goyangEkor();}
	}
}