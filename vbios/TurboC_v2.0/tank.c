/* edit:www.dotcpp.com */
#include <graphics.h>
#include <stdlib.h>
#include <dos.h>
#include <conio.h>
#include <bios.h>
#define KEY_ESC 0x01
#define KEY_SPACE 0x39
#define KEY_UP 0x48
#define KEY_LEFT 0x4b
#define KEY_RIGHT 0x4d
#define KEY_DOWN 0x50
 
int map[20][20]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
 1,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,2,2,2,2,0,0,2,2,2,2,0,0,0,0,0,0,0,1,
 1,0,0,0,0,0,0,0,2,0,0,2,0,1,1,1,1,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,2,2,2,2,2,2,2,0,0,0,0,0,0,0,2,2,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,0,1,
 1,0,1,1,1,1,3,3,3,3,0,0,0,0,0,0,0,2,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,3,3,3,0,1,
 1,0,0,0,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,0,0,0,3,3,3,1,1,1,1,1,1,1,0,0,0,0,1,
 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
 1,0,2,2,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,1,
 1,0,2,2,0,0,0,0,2,2,2,0,0,0,2,2,0,0,0,1,
 1,0,0,0,0,0,0,8,2,5,2,0,0,0,0,0,0,0,0,1,
 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
struct f
{
int x;
int y;
int direction;
};
struct play
{
int x;
int y;
int direction;
struct f fire[5];
int score;
}Playone;
struct a
{
int x;
int y;
int color;
int direction;
int directiontwo;
int fireplay;
struct f fire;
}amy[5];
char key_state[128],key_pressed[128];
void Init();
void End();
void DrawMap();
void DrawWater(int x,int y);
void DrawBrick(int x,int y);
void DrawTone(int x,int y);
void DrawHome(int x,int y);
void DrawBlack(int x,int y);
void DrawPlay(int x,int y);
void DrawAmy(int x,int y,int i);
void Score();
void GamePlay();
void GameOver();
void TimeDelay(unsigned long microsec);
int GetKey(int ScanCode);
void interrupt far (*OldInt9Handler)();
void far interrupt NewInt9();
void InstallKeyboard();
void ShutDownKeyboard();
void main(void)
{
Init();
DrawMap();
GamePlay();
End();
}
void TimeDelay(unsigned long microsec)
{
union REGS r;
r.h.ah=0x86;
r.x.cx=microsec>>16;
r.x.dx=microsec;
int86(0x15,&r,&r);
}
void Init()
{int gd=DETECT,gm;
initgraph(&gd,&gm,"C:\\TC20\\BGI");
cleardevice();
InstallKeyboard();
}
void End()
{
ShutDownKeyboard();
closegraph();
}
void DrawTone(int x,int y)
{
setfillstyle(SOLID_FILL,7);
bar(100+x*20-9,50+y*20-9,100+x*20+9,50+y*20+9);
}
void DrawWater(int x,int y)
{
setfillstyle(SOLID_FILL,BLUE);
bar(100+x*20-9,50+y*20-9,100+x*20+9,50+y*20+9);
}
void DrawBrick(int x,int y)
{
setfillstyle(SOLID_FILL,6);
bar(100+x*20-9,50+y*20-9,100+x*20+9,50+y*20+9);
setcolor(15);
line(100+x*20-9,50+y*20-4,100+x*20+9,50+y*20-4);
line(100+x*20-9,50+y*20+4,100+x*20+9,50+y*20+4);
line(100+x*20-4,50+y*20-9,100+x*20-4,50+y*20+9);
line(100+x*20+4,50+y*20-9,100+x*20+4,50+y*20+9);
}
void DrawHome(int x,int y)
{
setcolor(0);
setfillstyle(SOLID_FILL,GREEN);
fillellipse(100+x*20,50+y*20,9,9);
}
void DrawBlack(int x,int y)
{
setcolor(0);
setfillstyle(SOLID_FILL,0);
bar(100+x*20-9,50+y*20-9,100+x*20+9,50+y*20+9);
}
void DrawPlay(int x,int y)
{
setcolor(4);
circle(100+x*20,50+y*20,7);
switch(Playone.direction)
{
case 1:line(100+x*20,50+y*20,100+x*20,50+y*20-9);break;
case 2:line(100+x*20,50+y*20,100+x*20+9,50+y*20);break;
case 3:line(100+x*20,50+y*20,100+x*20,50+y*20+9);break;
case 4:line(100+x*20,50+y*20,100+x*20-9,50+y*20);break;
}
}
void DrawAmy(int x,int y,int i)
{
if(amy[i].color==12)
setcolor(12);
else if(amy[i].color==13)
setcolor(13);
else
setcolor(14);
circle(100+x*20,50+y*20,7);
switch(amy[i].direction)
{
case 1:line(100+x*20,50+y*20,100+x*20,50+y*20-9);break;
case 2:line(100+x*20,50+y*20,100+x*20+9,50+y*20);break;
case 3:line(100+x*20,50+y*20,100+x*20,50+y*20+9);break;
case 4:line(100+x*20,50+y*20,100+x*20-9,50+y*20);break;
}
}
void Score()
{
char s[10];
Playone.score+=10;
sprintf(s,"%d",Playone.score);
setfillstyle(SOLID_FILL,0);
bar(550,100,640,130);
settextstyle(0,0,2);
setcolor(YELLOW);
outtextxy(550,115,s);
}
void DrawMap()
{int i,j,k;
for(i=0;i<20;i++)
 {
 for(j=0;j<20;j++)
 if(map[i][j]==1)
 DrawTone(j,i);
 else if(map[i][j]==2)
 DrawBrick(j,i);
 else if(map[i][j]==3)
 DrawWater(j,i);
 else if(map[i][j]==5)
 DrawHome(j,i);
 else if(map[i][j]==8)
 {
 Playone.x=i;
 Playone.y=j;
 Playone.direction=1;
 DrawPlay(j,i);
 for(k=0;k<5;k++)
 Playone.fire[k].direction=-1;
 }
 else if(map[i][j]==9)
 {
 amy[0].x=1;amy[0].y=1;amy[0].direction=amy[0].directiontwo=3;
 amy[0].color=12;
 DrawAmy(j,i,0);
 }
 }
for(i=1;i<5;i++)
amy[i].direction=amy[i].fire.direction=-1;
outtextxy(210,450,"Edit By www.dotcpp.com");
settextstyle(0,0,2);
setcolor(9);
outtextxy(525,80,"Score");
setcolor(YELLOW);
outtextxy(550,115,"0");
}
void far interrupt NewInt9(void)
{
unsigned char ScanCode,temp;
ScanCode=inportb(0x60);
temp=inportb(0x61);
outportb(0x61,temp | 0x80);
outportb(0x61,temp & 0x7f);
if(ScanCode&0x80)
 {
 ScanCode&=0x7f;
 key_state[ScanCode]=0;
 }
else
 {
 key_state[ScanCode]=1;
key_pressed[ScanCode]=1;
 }
outportb(0x20,0x20);
}
 
void InstallKeyboard(void)
{
int i;
for(i=0;i<128;i++)
key_state[i]=key_pressed[i]=0;
OldInt9Handler=getvect(9); 
setvect(9,NewInt9); 
}
 
void ShutDownKeyboard(void)
{
setvect(9,OldInt9Handler);
}
 
int GetKey(int ScanCode)
{
int res;
res=key_state[ScanCode]|key_pressed[ScanCode];
key_pressed[ScanCode]=0;
return res;
}
void GameOver()
{
setcolor(0);
setfillstyle(SOLID_FILL,0);
fillellipse(100+9*20,50+18*20,9,9);
nosound();
setcolor(RED);
 settextstyle(0,0,4);
outtextxy(150,5,"GAME OVER");
while(1)
{
 if(GetKey(KEY_ESC))
 break;
}
}
void GamePlay()
{
int i,j,lose=0;
int t=0;
randomize();
while(1)
{
 for(i=0;i<5;i++)
 {
 if(amy[i].fire.direction>0)
 putpixel(100+amy[i].fire.y*20,50+amy[i].fire.x*20,11);
 }
 for(i=0;i<=4;i++)
 {
 if(Playone.fire[i].direction>0)
 putpixel(100+Playone.fire[i].y*20,50+Playone.fire[i].x*20,11);
 }
 TimeDelay(500000);
 for(i=0;i<5;i++)
 {
 if(amy[i].fire.direction>0)
 putpixel(100+amy[i].fire.y*20,50+amy[i].fire.x*20,0);
 }
 for(i=0;i<=4;i++)
 {
 if(Playone.fire[i].direction>0)
 putpixel(100+Playone.fire[i].y*20,50+Playone.fire[i].x*20,0);
 }
 for(i=0;i<=4;i++)
 {
 if(Playone.fire[i].direction<0)
 continue;
 if(Playone.fire[i].direction==1)
 {Playone.fire[i].x--;Playone.fire[i].y=Playone.fire[i].y;}
 else if(Playone.fire[i].direction==2)
 {Playone.fire[i].y++;Playone.fire[i].y=Playone.fire[i].y;}
 else if(Playone.fire[i].direction==3)
 {Playone.fire[i].x++;Playone.fire[i].y=Playone.fire[i].y;}
 else if(Playone.fire[i].direction==4)
 {Playone.fire[i].y--;Playone.fire[i].y=Playone.fire[i].y;}
 
 if(map[Playone.fire[i].x][Playone.fire[i].y]==1)
 Playone.fire[i].direction=-1;
 if(map[Playone.fire[i].x][Playone.fire[i].y]==2)
 {
 Playone.fire[i].direction=-1;
 DrawBlack(Playone.fire[i].y,Playone.fire[i].x);
 map[Playone.fire[i].x][Playone.fire[i].y]=0;
 }
 if(map[Playone.fire[i].x][Playone.fire[i].y]==5)
 {lose=1;break;}
 for(j=0;j<5;j++)
 {
 if(amy[j].direction<0)
 continue;
 if(amy[j].x==Playone.fire[i].x&&amy[j].y==Playone.fire[i].y)
 {
 Playone.fire[i].direction=-1;
 DrawBlack(Playone.fire[i].y,Playone.fire[i].x);
 map[Playone.fire[i].x][Playone.fire[i].y]=0;
 amy[j].fire.direction=amy[j].direction=-1;
 Score();
 }
 }
 }
for(i=0;i<5;i++)
{
 if(amy[i].direction<0||amy[i].fire.direction<0)
 continue;
 if(amy[i].fire.direction==1)
 {amy[i].fire.x--;amy[i].fire.y=amy[i].fire.y;}
 else if(amy[i].fire.direction==2)
 {amy[i].fire.y++;amy[i].fire.x=amy[i].fire.x;}
 else if(amy[i].fire.direction==3)
 {amy[i].fire.x++;amy[i].fire.y=amy[i].fire.y;}
 else if(amy[i].fire.direction==4)
 {amy[i].fire.y--;amy[i].fire.x=amy[i].fire.x;}
 
 if(map[amy[i].fire.x][amy[i].fire.y]==1)
 amy[i].fire.direction=-1;
 if(map[amy[i].fire.x][amy[i].fire.y]==2)
 {
 amy[i].fire.direction=-1;
 DrawBlack(amy[i].fire.y,amy[i].fire.x);
 map[amy[i].fire.x][amy[i].fire.y]=0;
 }
 if(map[amy[i].fire.x][amy[i].fire.y]==5)
 {lose=1;break;}
 if(amy[i].fire.x==Playone.x&&amy[i].fire.y==Playone.y)
 {
 for(j=0;j<5;j++)
 Playone.fire[j].direction=-1;
 amy[i].fire.direction=-1;
 DrawBlack(amy[i].fire.y,amy[i].fire.x);
 map[amy[i].fire.x][amy[i].fire.y]=0;
 lose=1;break;
 }
 }
 nosound();
 for(i=0;i<5;i++)
 {
 if(amy[i].direction<0)
 continue;
 while(1)
 {
 amy[i].directiontwo=random(4)+1;
 if(amy[i].direction==1&&amy[i].directiontwo==3)
 continue;
 if(amy[i].direction==3&&amy[i].directiontwo==1)
 continue;
 if(amy[i].direction==2&&amy[i].directiontwo==4)
 continue;
 if(amy[i].direction==4&&amy[i].directiontwo==2)
 continue;
 if(amy[i].directiontwo==3&&(map[amy[i].x+1][amy[i].y]==3||map[amy[i].x+1][amy[i].y]==1||map[amy[i].x+1][amy[i].y]==2))
 continue;
 if(amy[i].directiontwo==1&&(map[amy[i].x-1][amy[i].y]==3||map[amy[i].x-1][amy[i].y]==1||map[amy[i].x-1][amy[i].y]==2))
 continue;
 if(amy[i].directiontwo==2&&(map[amy[i].x][amy[i].y+1]==3||map[amy[i].x][amy[i].y+1]==1||map[amy[i].x][amy[i].y+1]==2))
 continue;
 if(amy[i].directiontwo==4&&(map[amy[i].x][amy[i].y-1]==3||map[amy[i].x][amy[i].y-1]==1||map[amy[i].x][amy[i].y-1]==2))
 continue;
 DrawBlack(amy[i].y,amy[i].x);
 amy[i].direction=amy[i].directiontwo;
 if(amy[i].direction==1)
 {amy[i].x--;amy[i].y=amy[i].y;}
 if(amy[i].direction==3)
 {amy[i].x++;amy[i].y=amy[i].y;}
 if(amy[i].direction==2)
 {amy[i].y++;amy[i].x=amy[i].x;}
 if(amy[i].direction==4)
 {amy[i].y--;amy[i].x=amy[i].x;}
 if(amy[i].x==Playone.x&&amy[i].y==Playone.y)
 lose=1;
 if(map[amy[i].x][amy[i].y]==5)
 lose=1;
 DrawAmy(amy[i].y,amy[i].x,i);
 if(amy[i].fire.direction<0)
 amy[i].fireplay=random(4);
 if(amy[i].fireplay==1&&amy[i].fire.direction<0)
 {
 amy[i].fire.direction=amy[i].direction;
 amy[i].fire.x=amy[i].x;
 amy[i].fire.y=amy[i].y;
 }
 break;
 }
 }
 if(lose)
 {GameOver();break;}
 if(GetKey(KEY_ESC))
 break;
 if(GetKey(KEY_UP))
 {
 if(Playone.direction==1&&map[Playone.x-1][Playone.y]!=1&&map[Playone.x-1][Playone.y]!=2)
 {
 if(map[Playone.x-1][Playone.y]==3)
 continue;
 DrawBlack(Playone.y,Playone.x);
 Playone.x--;
 Playone.direction=1;
 DrawPlay(Playone.y,Playone.x);
 }
 else
 {
 DrawBlack(Playone.y,Playone.x);
 Playone.direction=1;
 DrawPlay(Playone.y,Playone.x);
 }
 }
 else if(GetKey(KEY_DOWN))
 {
 if(Playone.direction==3&&map[Playone.x+1][Playone.y]!=1&&map[Playone.x+1][Playone.y]!=2)
 {
 if(map[Playone.x+1][Playone.y]==3)
 continue;
 DrawBlack(Playone.y,Playone.x);
 Playone.x++;
 Playone.direction=3;
 DrawPlay(Playone.y,Playone.x);
 }
 else
 {
 DrawBlack(Playone.y,Playone.x);
 Playone.direction=3;
 DrawPlay(Playone.y,Playone.x);
 }
 }
 if(GetKey(KEY_RIGHT))
 {
 if(Playone.direction==2&&map[Playone.x][Playone.y+1]!=1&&map[Playone.x][Playone.y+1]!=2)
 {
 if(map[Playone.x][Playone.y+1]==3)
 continue;
 DrawBlack(Playone.y,Playone.x);
 Playone.y++;
 Playone.direction=2;
 DrawPlay(Playone.y,Playone.x);
 }
 else
 {
 DrawBlack(Playone.y,Playone.x);
 Playone.direction=2;
 DrawPlay(Playone.y,Playone.x);
 }
 }
 if(GetKey(KEY_LEFT))
 {
 if(Playone.direction==4&&map[Playone.x][Playone.y-1]!=1&&map[Playone.x][Playone.y-1]!=2)
 {
 if(map[Playone.x][Playone.y-1]==3)
 continue;
 DrawBlack(Playone.y,Playone.x);
 Playone.y--;
 Playone.direction=4;
 DrawPlay(Playone.y,Playone.x);
 }
 else
 {
 DrawBlack(Playone.y,Playone.x);
 Playone.direction=4;
 DrawPlay(Playone.y,Playone.x);
 }
 }
 if(GetKey(KEY_SPACE))
 {
 for(i=0;i<5;i++)
 if(Playone.fire[i].direction<0)
 {
 sound(300);
 Playone.fire[i].direction=Playone.direction;
 Playone.fire[i].x=Playone.x;
 Playone.fire[i].y=Playone.y;
 break;
 }
 }
 if(map[Playone.x][Playone.y]==5)
 lose=1;
 for(i=0;i<5;i++)
 {
 if(amy[i].direction<0)
 continue;
 if(amy[i].x==Playone.x&&amy[i].y==Playone.y)
 lose=1;
 }
 if(lose)
 {GameOver();break;}
 t++;
 if(t==30)
 {t=0;
 for(i=0;i<5;i++)
 if(amy[i].direction<0)
 {
 amy[i].direction=amy[i].directiontwo=3;
 amy[i].x=1;
 amy[i].y=random(3);
 if(amy[i].y==0)
 amy[i].y=1;
 else if(amy[i].y==1)
 amy[i].y=9;
 else
 amy[i].y=18;
 amy[i].color=random(3)+12;
 DrawAmy(amy[i].y,amy[i].x,i);
 break;
 }
 }
}
}
