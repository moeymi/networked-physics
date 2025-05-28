#pragma once  
#include "pch.h"  
#include <numeric> // Include this header for std::iota  

struct UnionFind {  
   std::vector<int> parent, rank;  
   explicit UnionFind(size_t n) : parent(n), rank(n, 0) {  
       std::iota(parent.begin(), parent.end(), 0);  
   }  
   int find(int x) {  
       while (x != parent[x]) x = parent[x] = parent[parent[x]];  
       return x;  
   }  
   void unite(int a, int b) {  
       a = find(a);  b = find(b);  
       if (a == b) return;  
       if (rank[a] < rank[b]) parent[a] = b;  
       else if (rank[a] > rank[b]) parent[b] = a;  
       else { parent[b] = a; ++rank[a]; }  
   }  
};  

class IslandBuilder  
{  
};
