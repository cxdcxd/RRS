  // initialize optmizer
  optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);
  global_optimizer = nlopt::opt(nlopt::AUGLAG, ch*7);
  sub_optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);


  // add upper and lower bounds;
  lb.resize(ch*7);
  std::fill(lb.begin(),lb.end(),-0.17453);
  optimizer.set_lower_bounds(lb);
  global_optimizer.set_lower_bounds(lb);
  ub.resize(ch*7);
  std::fill(ub.begin(),ub.end(),0.17453);
  optimizer.set_upper_bounds(ub);
  global_optimizer.set_upper_bounds(ub);

  // set objective cost function to minimize
  optimizer.set_min_objective(costFunc,this);
  global_optimizer.set_min_objective(costFuncMultiThread,this);

  optimizer.set_ftol_rel(1e-6);
  optimizer.set_maxeval(maxeval);
  global_optimizer.set_ftol_rel(1e-6);
  sub_optimizer.set_ftol_rel(1e-6);
  global_optimizer.set_local_optimizer(sub_optimizer);
