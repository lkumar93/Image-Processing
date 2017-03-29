#include <iostream>

using namespace std;

int main(int argc, char** argv )
{
  int input_size = 25;
  int a[input_size] = {1,1,1,2,2,3,4,4,4,4,4,4,4,5,5,5,5,3,2,2,2,2,2,2,3};
  //int a[input_size] = {1,1,2,4,2,5,7,1,2,3,4,10,11,15,7,3,3,3,3,6,7,8,1,1};
  int prev_val = 0;
  int count = 0;
  int output_size=0;
  cout<<endl<<"Input"<<endl<<endl;
  for(int i = 0 ; i<input_size; i++)
 	cout<<a[i]<<",";
  cout<<endl<<endl<<"Output"<<endl<<endl;
  for(int i = 0 ; i<input_size; i++)
  {
	if(prev_val != a[i])
	{
		if(count != 0)
		{
			cout<<count<<" times - "<<prev_val<<endl;
			output_size += 2;
		}		
		count = 1;
	}
	else
	{
		++count;
	}

	prev_val = a[i];
  }

  cout<<count<<" times - "<<prev_val<<endl;
  output_size += 2;
  

  cout<<endl<<"Input Size = "<<input_size<<" , Output Size = "<<output_size<<endl;
  cout<<"Compression Ratio = "<<(float)input_size/output_size << " : 1"<<endl<<endl;

}
