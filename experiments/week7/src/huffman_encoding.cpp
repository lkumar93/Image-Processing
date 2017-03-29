#include<iostream>
#include <bits/stdc++.h>

#define MAX_ELEMENT 10	

using namespace std;

string encoded_data[MAX_ELEMENT+1] = {""};

//Data type used for Priority Queue
class Node
{
   int data;
   unsigned int frequency;


public:

   Node *left, *right;

   Node(int _data, unsigned int _frequency)
   {
      data = _data;
      frequency = _frequency;
   }
   int getData() const { return data; }
   unsigned int getFrequency() const { return frequency; }
};

//Comparator used by Priority Queue for sorting
class FrequencyComparator
{
public:
    int operator() (const Node* p1, const Node* p2)
    {
        return p1->getFrequency() > p2->getFrequency();
    }
};

//Encode bits as a string
void encode(Node* root, string str)
{
    if(root == NULL)
	return;
    if(root->getData() > -1)
	encoded_data[root->getData()] = str;

    encode(root->left, str+"0");
    encode(root->right,str+"1");

}

//Implement Huffman Encoding
void huffman_encode(int frequency_table[], int size)
{

  Node *left_node, *right_node, *top_node;

   // Creates a Min heap of nodes (order by Frequency)
  priority_queue<Node*, vector<Node*>, FrequencyComparator > MinHeap;

  for (int i = 0; i < size; i++)
  {
	if(frequency_table[i] > 0)
		MinHeap.push(new Node(i,frequency_table[i]) );
  }

  while(MinHeap.size() > 1)
  {
	left_node = MinHeap.top();
	MinHeap.pop();

	right_node = MinHeap.top();
	MinHeap.pop();

	// -1 denotes a special node containing frequencies of left and right nodes
	top_node = new Node(-1, left_node->getFrequency() + right_node->getFrequency());
	top_node->left = left_node;
	top_node->right = right_node;
	MinHeap.push(top_node);
  }

  encode(MinHeap.top(),"");

  for(int i = 0; i < size;i++)
  {
	if(frequency_table[i] > 0)
		cout<<" Data = "<< i <<" , Huffman Code = "<<encoded_data[i] <<" , Frequency = "<<frequency_table[i]<<endl;
  }
  
  cout<<endl;
}


 
int main ()
{
    int input_size = 25;

    int a[input_size] = {1,1,1,1,1,3,4,4,4,4,4,4,4,5,5,5,5,3,7,1,1,1,2,2,9};
  
    int frequency_table[MAX_ELEMENT+1] = {0};

    cout<<endl<<"Input"<<endl<<endl;
    
    for(int i = 0 ; i<input_size; i++)
 	cout<<a[i]<<",";

    for(int i = 0; i < input_size; i++)
    {
	frequency_table[a[i]] += 1;
    }
  
    cout<<endl<<endl<<"Output - Bit Representation"<<endl;

    huffman_encode(frequency_table, MAX_ELEMENT+1);
 
    return 0;
}
