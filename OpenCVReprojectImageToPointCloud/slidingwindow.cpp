#include<>

int main()
{

Mat img;
int img_rows,_img_cols;
Mat temp;
imt temp_rows,temp_cols;
int x,y;
//intelligently guess two values : upper bound of threshold,and lowerbound of threshold
double delta_t1,delta_t2;
int delta_x_max=3,delta_x_nom=2,delta_x_min=1;
int delta_y_max=3,delta_y_min=1,delta_y_nom=2; //3 2 1 was there in the paper
 // we assume the x and y params are same which i dont think would be a problem
int delta_x; //the amount by which the window moves right
vector delta_y; // an array to ensure that sliding window moves by the same amount in all rows

int k;

	for(scalefactor=1;scalefactor;scalefactor*=k)
		for (y=0;y<img_rows-temp_rows;)
			{
			delta_x=1; //one pixel
			for (x=0;x<img_cols-temp_cols;)
				{		
				delta_x=delta_x-1; //Doing this to reduce the scale 
					delta_y[x]=delta_y[x]-1; //ditto
					//now executing the detector
					if (delta_x=0 and delta_y[x]=0)
						//scale the image here
						img_temp= img(x+temp_cols,y+temp_rows); //extract corresponding image
						exit_stage=FaceDetect(img_temp); //Process Based on return value ie loss function
						if (exit_stage<=delta_t2)
							{
							
							rectangle(x,x+temp_cols,y,y+temp_rows);//Draw Bounding Box
							//Push into vector of faces
							delta_x=delta_x_min; //let us confirm the fact that a face is here
							delta_y[x]=delta_y_min;
							}
						else if (exit_stage<delta_t1)&&(exit_stage>delta_t2) //some possiblity loss fn is somewhere between face and min(non face)=
							{
							delta_x=delta_x_nom;
							delta_y[x]=delta_y_nom; 
							}
						else if (exit_stage>delta_t1) //ie no possiblity of being a face ie loss>max(face) ->basically our lowerbound of threshold
							{
							delta_x=delta_x_max; //move by maximum
							delta_y=delta_y_max;
							}
					else if (delta_x=0)
						delta_x=delta_x_min;
					else if  (delta_y[x]=0)
						delta_y[x]=delta_x_min;
					x+=delta_x;
					y+=delta_y[x];
				}	//end of X loop
			}		//end of y loop
	} //end of scale factor loop 					