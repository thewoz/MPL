/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2024
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _H_MPL_GRAPH_H_
#define _H_MPL_GRAPH_H_

#include <cstdio>
#include <cstdlib>

#include <vector>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************
  // namespace graph
  //****************************************************************************
  namespace graph {

    //****************************************************************************
    // class node_t
    //****************************************************************************
    class node_t {

    public:
      
      int frame;
            
      std::vector<int> parents;
      std::vector<int> children;
      
      node_t() : frame(-1) { parents.clear(); children.clear(); }
      
      node_t(int _frame) : frame(_frame) { parents.clear(); children.clear(); }
      
      //****************************************************************************
      //  isRoot
      //****************************************************************************
      inline bool isRoot() {
        if(sizeParents() > 0) return false;
        else return true;
      }
      
      //****************************************************************************
      //  isTail
      //****************************************************************************
      inline bool isTail() {
        if(sizeChildren()==0) return true;
        else return false;
      }
      
      //****************************************************************************
      //  addParents
      //****************************************************************************
      inline void addParents(int id) { parents.push_back(id); }
      
      //****************************************************************************
      //  addChildren
      //****************************************************************************
      inline void addChildren(int id) { children.push_back(id); }
      
      //****************************************************************************
      //  size
      //****************************************************************************
      inline size_t sizeParents()  const { return parents.size();  }
      inline size_t sizeChildren() const { return children.size(); }
      
      //****************************************************************************
      //  operator []
      //****************************************************************************
      inline const int & operator [] (const int index) const { return children[index]; }
      inline       int & operator [] (const int index)       { return children[index]; }
      
    };

  } /* namespace graph */
  

  //****************************************************************************
  //  graph_t
  //****************************************************************************
  class graph_t {
    
  protected:
    
    // vettore dei nodi
    std::vector<graph::node_t *> nodes;
    
    std::vector<bool> isGood;

    bool isInited;
    
  public:
    
    //****************************************************************************
    //  graph_t()
    //****************************************************************************
    graph_t() : isInited(false) { }
    
    //****************************************************************************
    //  addNode
    //****************************************************************************
    void addNode(graph::node_t * node) { nodes.push_back(node); isGood.push_back(false); }
    
    //****************************************************************************
    //  createTracks
    //****************************************************************************
    void createTracks(std::vector<std::vector<int>> & tracks) {

      if(!isInited) {
        fprintf(stderr, "error the graph should be inited before call createTracks()\n");
        abort();
      }
      
      // pulisco le traccie
      tracks.clear();
      
      // ciclo su tutti i nodi
      for(int i=0; i<nodes.size(); ++i){
      
        // se il nodo è buono o non è una radice lo salto
        if(!isGood[i] || !nodes[i]->isRoot()) continue;

        // creo tutte le traiettorie a partire da quel punto
        buildTracksForward(i, std::vector<int>(), tracks);

      }
      
    }
    
    //****************************************************************************
    //  isToAdd
    //****************************************************************************
    virtual inline bool isToAdd(const std::vector<int> & track) { return true; }
    
  private:
      
    //****************************************************************************
    //  buildTracksForward
    //****************************************************************************
    void buildTracksForward(int id, std::vector<int> track, std::vector<std::vector<int>> & tracks) {
          
      // mi segno che sono passato da questo nodo
      isGood[id] = false;
      
      // aggiungo il nodo alla traccia
      track.push_back(id);
      
      // se il nodo non ha figli il cammino e' finito
      if(nodes[id]->isTail()) {
        if(isToAdd(track)) tracks.push_back(track);
        return;
      }
      
      // visito tutti i figli del nodo
      for(int i=0; i<nodes[id]->sizeChildren(); ++i)
        if(isGood[nodes[id]->children[i]]) // se vanno visitati
          buildTracksForward(nodes[id]->children[i], track, tracks);
      
    }
    
  public:
    
    //****************************************************************************/
    // save
    //****************************************************************************/
    void save(const std::string & filepath) {
      
      if(!isInited) {
        fprintf(stderr, "error the graph should be inited before call save()\n");
        abort();
      }
      
      // dot filename.gv -Tpdf -o filename.pdf
      
      FILE * outFile = fopen(filepath.c_str(), "w");
      if(outFile == NULL) {
        fprintf(stderr, "error in open the output file in dump graph\n");
        abort();
      }
      
      fprintf(outFile, "digraph dump {\n");
      
      fprintf(outFile, "\n\n");
      
      std::map<int,std::vector<int>> rank;
      
      for(int i=0; i<nodes.size(); i++) {
        
        if(!isGood[i]) continue;
        
        rank[nodes[i]->frame].push_back(i);
        
        fprintf(outFile, "%d [label=\"%d\"];\n", i, i);

      }
      
      fprintf(outFile,"\n\n");
      
      for(auto rankIt=rank.begin(); rankIt!=rank.end(); rankIt++) {
        
        fprintf(outFile,"\t {rank=same;");
        
        for(int j=0; j<rankIt->second.size(); j++) fprintf(outFile," %d",rankIt->second[j]);
        
        fprintf(outFile,";}\n\n");
        
      }
      
      fprintf(outFile,"\n\n");
      
      for(int i=0; i<nodes.size(); i++) {
        for(int j=0; j<nodes[i]->sizeChildren(); ++j) {
          if(isGood[nodes[i]->children[j]]) fprintf(outFile, "%d -> %d \n", i, nodes[i]->children[j]);
        }
      }
      
      fprintf(outFile, "}\n");
      
      fclose(outFile);
      
    }
    
    
  };

} /* namespace mpl */
  
#endif /* _H_MPL_GRAPH_H_ */
