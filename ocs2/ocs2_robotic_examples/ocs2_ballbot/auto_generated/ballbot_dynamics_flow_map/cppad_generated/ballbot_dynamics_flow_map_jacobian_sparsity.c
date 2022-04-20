void ballbot_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                 unsigned long const** col,
                                                 unsigned long* nnz) {
   static unsigned long const rows[60] = {0,1,2,3,4,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,9};
   static unsigned long const cols[60] = {6,7,8,9,10,3,4,5,6,7,8,9,10,11,12,13,3,4,5,6,7,8,9,10,11,12,13,3,4,5,6,7,8,9,10,11,12,13,3,4,5,6,7,8,9,10,11,12,13,3,4,5,6,7,8,9,10,11,12,13};
   *row = rows;
   *col = cols;
   *nnz = 60;
}
