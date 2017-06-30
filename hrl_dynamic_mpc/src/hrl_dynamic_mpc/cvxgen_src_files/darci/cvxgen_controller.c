#include<boost/python.hpp>
#include "solver.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

int flags = 0;

// user defined functions for boost python to interpret
boost::python::list runController(double alpha,
				  double beta,
				  double kappa,
				  double zeta,
				  double mu,
				  double xyz_weight,
				  double posture_weight,
				  boost::python::list delta_x_des,
				  boost::python::list delta_q_des,
				  boost::python::list J,
				  boost::python::list A_tl,
				  boost::python::list A_tr,
				  boost::python::list A_bl,
				  boost::python::list A_br,
				  boost::python::list B_t1,
				  boost::python::list B_t2,
				  boost::python::list B_t3,
				  boost::python::list B_b1,
				  boost::python::list B_b2,
				  boost::python::list B_b3,
				  boost::python::list q_0,
				  boost::python::list qd_0,
				  boost::python::list q_des_cur_0,
				  boost::python::list q_min,
				  boost::python::list q_max,
				  boost::python::list torque_max,
				  boost::python::list torque_min,
				  boost::python::list Kp,
				  boost::python::list Kd,
				  boost::python::list tau_max_delta_t,
				  boost::python::list tau_cont_sum_0,
				  boost::python::list mass,
				  boost::python::list delta_f_max,
				  boost::python::list n_K_J_all)
{
  //printf("got to start");

  if (flags == 0)
    {
      set_defaults();
      setup_indexing();
      flags = 1;
      settings.verbose = 0;
    }

  //printf("got past flags");

  params.alpha[0] = alpha;
  params.beta[0] = beta;
  params.kappa[0] = kappa;
  params.zeta[0] = zeta;
  params.xyz_weight[0] = xyz_weight;
  params.posture_weight[0] = posture_weight;
  //params.u_slew_max[0] = u_slew_max;
  params.mu[0] = mu;

  //printf("got past scalars");
  int ii;
  for (ii=0; ii < boost::python::len(delta_x_des); ii++)
    {
      //printf("delta_x_des %d is %0.5f \n", ii, delta_x_des[ii]);
      params.delta_x_des[ii] = boost::python::extract<double>(delta_x_des[ii]);
    }

  for (ii=0; ii < boost::python::len(delta_q_des); ii++)
    {
      //printf("delta_q_des %d is %0.5f \n", ii, delta_q_des[ii]);
      params.delta_q_des[ii] = boost::python::extract<double>(delta_q_des[ii]);
    }

  //printf("past_x_d");
  for (ii=0; ii < boost::python::len(J); ii++)
    {
      //printf("J %d is %0.5f \n", ii, J[ii]);
      params.J[ii] = boost::python::extract<double>(J[ii]);
    }

  //printf("past_J yo");
  for (ii=0; ii < boost::python::len(A_tl); ii++)
    {
      params.A_tl[ii] = boost::python::extract<double>(A_tl[ii]);
      params.A_tr[ii] = boost::python::extract<double>(A_tr[ii]);
      params.A_bl[ii] = boost::python::extract<double>(A_bl[ii]);
      params.A_br[ii] = boost::python::extract<double>(A_br[ii]);
    }

  for (ii=0; ii < boost::python::len(B_t1); ii++)
    {
      params.B_t1[ii] = boost::python::extract<double>(B_t1[ii]);
      params.B_t2[ii] = boost::python::extract<double>(B_t2[ii]);
      params.B_t3[ii] = boost::python::extract<double>(B_t3[ii]);
      params.B_b1[ii] = boost::python::extract<double>(B_b1[ii]);
      params.B_b2[ii] = boost::python::extract<double>(B_b2[ii]);
      params.B_b3[ii] = boost::python::extract<double>(B_b3[ii]);
    }

  //printf("past A and B");
  for (ii=0; ii < boost::python::len(q_0); ii++)
    {
      //printf("index is : %i \n", ii);
      //printf("length is : %i \n", boost::python::len(q_0));
      params.q_min[ii] = boost::python::extract<double>(q_min[ii]);
      //printf("past q_min \n");
      //params.u_0[ii] = u_0[ii];
      //params.u_prev[ii] = boost::python::extract<double>(u_prev[ii]);
      params.torque_max[ii] = boost::python::extract<double>(torque_max[ii]);
      params.torque_min[ii] = boost::python::extract<double>(torque_min[ii]);
      params.q_0[ii] = boost::python::extract<double>(q_0[ii]);
      params.qd_0[ii] = boost::python::extract<double>(qd_0[ii]);
      params.q_des_cur_0[ii] = boost::python::extract<double>(q_des_cur_0[ii]);

      //printf("q_min is %0.5f \n", (double)boost::python::extract<double>(q_min[ii]));
      //printf("q_max is %0.5f \n", (double)boost::python::extract<double>(q_max[ii]));
      //printf("q_0 is %0.5f \n", (double)boost::python::extract<double>(q_0[ii]));

      //params.q_min[ii] = boost::python::extract<double>(q_min[ii]);
      params.q_max[ii] = boost::python::extract<double>(q_max[ii]);
      //printf("past q_max \n");
      params.tau_max_delta_t[ii] = boost::python::extract<double>(tau_max_delta_t[ii]);
      //printf("past tau_max_delta \n");
      params.tau_cont_sum_0[ii] = boost::python::extract<double>(tau_cont_sum_0[ii]);
      //printf("past tau_cont \n");
    }

  //printf("q's and other");

  for (ii=0; ii < boost::python::len(mass); ii++)
    {
      params.mass[ii] = boost::python::extract<double>(mass[ii]);
      params.Kp[ii] = boost::python::extract<double>(Kp[ii]);
      params.Kd[ii] = boost::python::extract<double>(Kd[ii]);
      //params.vel_norm_J[ii] = state_data.vel_norm_J[ii];
    }

  //printf("past mass and kp, kd");

  for (ii=0; ii < boost::python::len(delta_f_max); ii++)
    {
      params.delta_f_max[ii] = boost::python::extract<double>(delta_f_max[ii]);
    }

  //printf("past delta_f_max");
  for (ii=0; ii < boost::python::len(n_K_J_all); ii++)
    {
      params.n_K_J_all[ii] = boost::python::extract<double>(n_K_J_all[ii]);
      //params.n_J_all[ii] = state_data.n_J_all[ii];
    }

  //printf("up to solve");

  solve();

  //printf("past solve");

  boost::python::list u_0;

  ////// THE HARD CODED PARAMETER BELOW NEEDS TO BE UPDATED DEPENDING ON THE ROBOT //////////////
  if (work.converged == 1)
    {
      int kk;
      for (kk = 0; kk < 3; kk++)
  	{
  	  boost::python::list buff;
  	  for (ii =0; ii< boost::python::len(q_0); ii++)
  	    {
  	      buff.append(vars.u[kk][ii]);
  	      //printf("u %d is : %.5f \n", ii, vars.u[0][ii]);
  	    }
  	  u_0.append(buff);
  	}
    }
  else
    {
      boost::python::list buff;
      for (ii =0; ii< boost::python::len(q_0); ii++)
  	{
  	  //printf("u %d is : %.5f \n", ii, 0.0);
  	  buff.append(0.0);
  	}
      u_0.append(buff);
    }

  return u_0;
}

int return_stuff(boost::python::list& thing)
{
  set_defaults();
  setup_indexing();
  settings.verbose = 0;
  load_default_data();

  solve();

  int answer = boost::python::extract<int>(thing[-1]);
  return answer;
}

char const* greet()
{
  printf("got into greet");
  return "hello, world (and Ari)";
}

// this is what tells boost what the function names and definitions should be (DON'T DELETE)
BOOST_PYTHON_MODULE(cvxgen_controller)
{
  using namespace boost::python;
  def("greet", greet);
  def("return_stuff", return_stuff);
  def("runController", runController);
}

void load_default_data(void) {
  /* params.kappa[0] = 1.101595805149151; */
  /* /\* Make this a diagonal PSD matrix, even though it's not diagonal. *\/ */
  /* params.mass[0] = 1.7081478226181048; */
  /* params.mass[4] = 0; */
  /* params.mass[8] = 0; */
  /* params.mass[12] = 0; */
  /* params.mass[1] = 0; */
  /* params.mass[5] = 1.2909047389129444; */
  /* params.mass[9] = 0; */
  /* params.mass[13] = 0; */
  /* params.mass[2] = 0; */
  /* params.mass[6] = 0; */
  /* params.mass[10] = 1.510827605197663; */
  /* params.mass[14] = 0; */
  /* params.mass[3] = 0; */
  /* params.mass[7] = 0; */
  /* params.mass[11] = 0; */
  /* params.mass[15] = 1.8929469543476547; */
  /* params.tau_max_delta_t[0] = 1.5851723557337523; */
  /* params.tau_max_delta_t[1] = -1.497658758144655; */
  /* params.tau_max_delta_t[2] = -1.171028487447253; */
  /* params.tau_max_delta_t[3] = -1.7941311867966805; */
  /* params.alpha[0] = 0.8816196873012729; */
  /* params.delta_x_d[0] = -1.8804951564857322; */
  /* params.delta_x_d[1] = -0.17266710242115568; */
  /* params.J[0] = 0.596576190459043; */
  /* params.J[1] = -0.8860508694080989; */
  /* params.J[2] = 0.7050196079205251; */
  /* params.J[3] = 0.3634512696654033; */
  /* params.J[4] = -1.9040724704913385; */
  /* params.J[5] = 0.23541635196352795; */
  /* params.J[6] = -0.9629902123701384; */
  /* params.J[7] = -0.3395952119597214; */
  /* params.q_0[0] = -0.865899672914725; */
  /* params.q_0[1] = 0.7725516732519853; */
  /* params.q_0[2] = -0.23818512931704205; */
  /* params.q_0[3] = -1.372529046100147; */
  /* params.mu[0] = 1.0892980360636895; */
  /* params.beta[0] = 1.560629529022734; */
  /* params.n_K_J_all[0] = -0.774545870495281; */
  /* params.n_K_J_all[1] = -1.1121684642712744; */
  /* params.n_K_J_all[2] = -0.44811496977740495; */
  /* params.n_K_J_all[3] = 1.7455345994417217; */
  /* params.n_K_J_all[4] = 1.9039816898917352; */
  /* params.n_K_J_all[5] = 0.6895347036512547; */
  /* params.n_K_J_all[6] = 1.6113364341535923; */
  /* params.n_K_J_all[7] = 1.383003485172717; */
  /* params.n_K_J_all[8] = -0.48802383468444344; */
  /* params.n_K_J_all[9] = -1.631131964513103; */
  /* params.n_K_J_all[10] = 0.6136436100941447; */
  /* params.n_K_J_all[11] = 0.2313630495538037; */
  /* params.n_K_J_all[12] = -0.5537409477496875; */
  /* params.n_K_J_all[13] = -1.0997819806406723; */
  /* params.n_K_J_all[14] = -0.3739203344950055; */
  /* params.n_K_J_all[15] = -0.12423900520332376; */
  /* params.n_K_J_all[16] = -0.923057686995755; */
  /* params.n_K_J_all[17] = -0.8328289030982696; */
  /* params.n_K_J_all[18] = -0.16925440270808823; */
  /* params.n_K_J_all[19] = 1.442135651787706; */
  /* params.n_K_J_all[20] = 0.34501161787128565; */
  /* params.n_K_J_all[21] = -0.8660485502711608; */
  /* params.n_K_J_all[22] = -0.8880899735055947; */
  /* params.n_K_J_all[23] = -0.1815116979122129; */
  /* params.n_K_J_all[24] = -1.17835862158005; */
  /* params.n_K_J_all[25] = -1.1944851558277074; */
  /* params.n_K_J_all[26] = 0.05614023926976763; */
  /* params.n_K_J_all[27] = -1.6510825248767813; */
  /* params.n_K_J_all[28] = -0.06565787059365391; */
  /* params.n_K_J_all[29] = -0.5512951504486665; */
  /* params.n_K_J_all[30] = 0.8307464872626844; */
  /* params.n_K_J_all[31] = 0.9869848924080182; */
  /* params.n_K_J_all[32] = 0.7643716874230573; */
  /* params.n_K_J_all[33] = 0.7567216550196565; */
  /* params.n_K_J_all[34] = -0.5055995034042868; */
  /* params.n_K_J_all[35] = 0.6725392189410702; */
  /* params.n_K_J_all[36] = -0.6406053441727284; */
  /* params.n_K_J_all[37] = 0.29117547947550015; */
  /* params.n_K_J_all[38] = -0.6967713677405021; */
  /* params.n_K_J_all[39] = -0.21941980294587182; */
  /* params.delta_f_max[0] = -1.753884276680243; */
  /* params.delta_f_max[1] = -1.0292983112626475; */
  /* params.delta_f_max[2] = 1.8864104246942706; */
  /* params.delta_f_max[3] = -1.077663182579704; */
  /* params.delta_f_max[4] = 0.7659100437893209; */
  /* params.delta_f_max[5] = 0.6019074328549583; */
  /* params.delta_f_max[6] = 0.8957565577499285; */
  /* params.delta_f_max[7] = -0.09964555746227477; */
  /* params.delta_f_max[8] = 0.38665509840745127; */
  /* params.delta_f_max[9] = -1.7321223042686946; */
  /* params.zeta[0] = 0.14512427564446684; */
  /* params.delta_rate_f_max[0] = -1.2040958948116867; */
  /* params.delta_rate_f_max[1] = -1.3925560119658358; */
  /* params.delta_rate_f_max[2] = -1.5995826216742213; */
  /* params.delta_rate_f_max[3] = -1.4828245415645833; */
  /* params.delta_rate_f_max[4] = 0.21311092723061398; */
  /* params.delta_rate_f_max[5] = -1.248740700304487; */
  /* params.delta_rate_f_max[6] = 1.808404972124833; */
  /* params.delta_rate_f_max[7] = 0.7264471152297065; */
  /* params.delta_rate_f_max[8] = 0.16407869343908477; */
  /* params.delta_rate_f_max[9] = 0.8287224032315907; */
  /* params.A_tl[0] = -0.9444533161899464; */
  /* params.A_tl[1] = 1.7069027370149112; */
  /* params.A_tl[2] = 1.3567722311998827; */
  /* params.A_tl[3] = 0.9052779937121489; */
  /* params.A_tl[4] = -0.07904017565835986; */
  /* params.A_tl[5] = 1.3684127435065871; */
  /* params.A_tl[6] = 0.979009293697437; */
  /* params.A_tl[7] = 0.6413036255984501; */
  /* params.A_tl[8] = 1.6559010680237511; */
  /* params.A_tl[9] = 0.5346622551502991; */
  /* params.A_tl[10] = -0.5362376605895625; */
  /* params.A_tl[11] = 0.2113782926017822; */
  /* params.A_tl[12] = -1.2144776931994525; */
  /* params.A_tl[13] = -1.2317108144255875; */
  /* params.A_tl[14] = 0.9026784957312834; */
  /* params.A_tl[15] = 1.1397468137245244; */
  /* params.qd_0[0] = 1.8883934547350631; */
  /* params.qd_0[1] = 1.4038856681660068; */
  /* params.qd_0[2] = 0.17437730638329096; */
  /* params.qd_0[3] = -1.6408365219077408; */
  /* params.A_tr[0] = -0.04450702153554875; */
  /* params.A_tr[1] = 1.7117453902485025; */
  /* params.A_tr[2] = 1.1504727980139053; */
  /* params.A_tr[3] = -0.05962309578364744; */
  /* params.A_tr[4] = -0.1788825540764547; */
  /* params.A_tr[5] = -1.1280569263625857; */
  /* params.A_tr[6] = -1.2911464767927057; */
  /* params.A_tr[7] = -1.7055053231225696; */
  /* params.A_tr[8] = 1.56957275034837; */
  /* params.A_tr[9] = 0.5607064675962357; */
  /* params.A_tr[10] = -1.4266707301147146; */
  /* params.A_tr[11] = -0.3434923211351708; */
  /* params.A_tr[12] = -1.8035643024085055; */
  /* params.A_tr[13] = -1.1625066019105454; */
  /* params.A_tr[14] = 0.9228324965161532; */
  /* params.A_tr[15] = 0.6044910817663975; */
  /* params.B_t1[0] = -0.0840868104920891; */
  /* params.B_t1[1] = -0.900877978017443; */
  /* params.B_t1[2] = 0.608892500264739; */
  /* params.B_t1[3] = 1.8257980452695217; */
  /* params.B_t1[4] = -0.25791777529922877; */
  /* params.B_t1[5] = -1.7194699796493191; */
  /* params.B_t1[6] = -1.7690740487081298; */
  /* params.B_t1[7] = -1.6685159248097703; */
  /* params.B_t1[8] = 1.8388287490128845; */
  /* params.B_t1[9] = 0.16304334474597537; */
  /* params.B_t1[10] = 1.3498497306788897; */
  /* params.B_t1[11] = -1.3198658230514613; */
  /* params.B_t1[12] = -0.9586197090843394; */
  /* params.B_t1[13] = 0.7679100474913709; */
  /* params.B_t1[14] = 1.5822813125679343; */
  /* params.B_t1[15] = -0.6372460621593619; */
  /* params.q_des_cur_0[0] = -1.741307208038867; */
  /* params.q_des_cur_0[1] = 1.456478677642575; */
  /* params.q_des_cur_0[2] = -0.8365102166820959; */
  /* params.q_des_cur_0[3] = 0.9643296255982503; */
  /* params.B_t2[0] = -1.367865381194024; */
  /* params.B_t2[1] = 0.7798537405635035; */
  /* params.B_t2[2] = 1.3656784761245926; */
  /* params.B_t2[3] = 0.9086083149868371; */
  /* params.B_t2[4] = -0.5635699005460344; */
  /* params.B_t2[5] = 0.9067590059607915; */
  /* params.B_t2[6] = -1.4421315032701587; */
  /* params.B_t2[7] = -0.7447235390671119; */
  /* params.B_t2[8] = -0.32166897326822186; */
  /* params.B_t2[9] = 1.5088481557772684; */
  /* params.B_t2[10] = -1.385039165715428; */
  /* params.B_t2[11] = 1.5204991609972622; */
  /* params.B_t2[12] = 1.1958572768832156; */
  /* params.B_t2[13] = 1.8864971883119228; */
  /* params.B_t2[14] = -0.5291880667861584; */
  /* params.B_t2[15] = -1.1802409243688836; */
  /* params.tau_cont_sum_0[0] = -1.037718718661604; */
  /* params.tau_cont_sum_0[1] = 1.3114512056856835; */
  /* params.tau_cont_sum_0[2] = 1.8609125943756615; */
  /* params.tau_cont_sum_0[3] = 0.7952399935216938; */
  /* params.B_t3[0] = -0.07001183290468038; */
  /* params.B_t3[1] = -0.8518009412754686; */
  /* params.B_t3[2] = 1.3347515373726386; */
  /* params.B_t3[3] = 1.4887180335977037; */
  /* params.B_t3[4] = -1.6314736327976336; */
  /* params.B_t3[5] = -1.1362021159208933; */
  /* params.B_t3[6] = 1.327044361831466; */
  /* params.B_t3[7] = 1.3932155883179842; */
  /* params.B_t3[8] = -0.7413880049440107; */
  /* params.B_t3[9] = -0.8828216126125747; */
  /* params.B_t3[10] = -0.27673991192616; */
  /* params.B_t3[11] = 0.15778600105866714; */
  /* params.B_t3[12] = -1.6177327399735457; */
  /* params.B_t3[13] = 1.3476485548544606; */
  /* params.B_t3[14] = 0.13893948140528378; */
  /* params.B_t3[15] = 1.0998712601636944; */
  /* params.A_bl[0] = -1.0766549376946926; */
  /* params.A_bl[1] = 1.8611734044254629; */
  /* params.A_bl[2] = 1.0041092292735172; */
  /* params.A_bl[3] = -0.6276245424321543; */
  /* params.A_bl[4] = 1.794110587839819; */
  /* params.A_bl[5] = 0.8020471158650913; */
  /* params.A_bl[6] = 1.362244341944948; */
  /* params.A_bl[7] = -1.8180107765765245; */
  /* params.A_bl[8] = -1.7774338357932473; */
  /* params.A_bl[9] = 0.9709490941985153; */
  /* params.A_bl[10] = -0.7812542682064318; */
  /* params.A_bl[11] = 0.0671374633729811; */
  /* params.A_bl[12] = -1.374950305314906; */
  /* params.A_bl[13] = 1.9118096386279388; */
  /* params.A_bl[14] = 0.011004190697677885; */
  /* params.A_bl[15] = 1.3160043138989015; */
  /* params.A_br[0] = -1.7038488148800144; */
  /* params.A_br[1] = -0.08433819112864738; */
  /* params.A_br[2] = -1.7508820783768964; */
  /* params.A_br[3] = 1.536965724350949; */
  /* params.A_br[4] = -0.21675928514816478; */
  /* params.A_br[5] = -1.725800326952653; */
  /* params.A_br[6] = -1.6940148707361717; */
  /* params.A_br[7] = 0.15517063201268; */
  /* params.A_br[8] = -1.697734381979077; */
  /* params.A_br[9] = -1.264910727950229; */
  /* params.A_br[10] = -0.2545716633339441; */
  /* params.A_br[11] = -0.008868675926170244; */
  /* params.A_br[12] = 0.3332476609670296; */
  /* params.A_br[13] = 0.48205072561962936; */
  /* params.A_br[14] = -0.5087540014293261; */
  /* params.A_br[15] = 0.4749463319223195; */
  /* params.B_b1[0] = -1.371021366459455; */
  /* params.B_b1[1] = -0.8979660982652256; */
  /* params.B_b1[2] = 1.194873082385242; */
  /* params.B_b1[3] = -1.3876427970939353; */
  /* params.B_b1[4] = -1.106708108457053; */
  /* params.B_b1[5] = -1.0280872812241797; */
  /* params.B_b1[6] = -0.08197078070773234; */
  /* params.B_b1[7] = -1.9970179118324083; */
  /* params.B_b1[8] = -1.878754557910134; */
  /* params.B_b1[9] = -0.15380739340877803; */
  /* params.B_b1[10] = -1.349917260533923; */
  /* params.B_b1[11] = 0.7180072150931407; */
  /* params.B_b1[12] = 1.1808183487065538; */
  /* params.B_b1[13] = 0.31265343495084075; */
  /* params.B_b1[14] = 0.7790599086928229; */
  /* params.B_b1[15] = -0.4361679370644853; */
  /* params.B_b2[0] = -1.8148151880282066; */
  /* params.B_b2[1] = -0.24231386948140266; */
  /* params.B_b2[2] = -0.5120787511622411; */
  /* params.B_b2[3] = 0.3880129688013203; */
  /* params.B_b2[4] = -1.4631273212038676; */
  /* params.B_b2[5] = -1.0891484131126563; */
  /* params.B_b2[6] = 1.2591296661091191; */
  /* params.B_b2[7] = -0.9426978934391474; */
  /* params.B_b2[8] = -0.358719180371347; */
  /* params.B_b2[9] = 1.7438887059831263; */
  /* params.B_b2[10] = -0.8977901479165817; */
  /* params.B_b2[11] = -1.4188401645857445; */
  /* params.B_b2[12] = 0.8080805173258092; */
  /* params.B_b2[13] = 0.2682662017650985; */
  /* params.B_b2[14] = 0.44637534218638786; */
  /* params.B_b2[15] = -1.8318765960257055; */
  /* params.B_b3[0] = -0.3309324209710929; */
  /* params.B_b3[1] = -1.9829342633313622; */
  /* params.B_b3[2] = -1.013858124556442; */
  /* params.B_b3[3] = 0.8242247343360254; */
  /* params.B_b3[4] = -1.753837136317201; */
  /* params.B_b3[5] = -0.8212260055868805; */
  /* params.B_b3[6] = 1.9524510112487126; */
  /* params.B_b3[7] = 1.884888920907902; */
  /* params.B_b3[8] = -0.0726144452811801; */
  /* params.B_b3[9] = 0.9427735461129836; */
  /* params.B_b3[10] = 0.5306230967445558; */
  /* params.B_b3[11] = -0.1372277142250531; */
  /* params.B_b3[12] = 1.4282657305652786; */
  /* params.B_b3[13] = -1.309926991335284; */
  /* params.B_b3[14] = 1.3137276889764422; */
  /* params.B_b3[15] = -1.8317219061667278; */
  /* params.q_min[0] = 1.4678147672511939; */
  /* params.q_min[1] = 0.703986349872991; */
  /* params.q_min[2] = -0.2163435603565258; */
  /* params.q_min[3] = 0.6862809905371079; */
  /* params.q_max[0] = -0.15852598444303245; */
  /* params.q_max[1] = 1.1200128895143409; */
  /* params.q_max[2] = -1.5462236645435308; */
  /* params.q_max[3] = 0.0326297153944215; */
  /* params.u_max[0] = 1.7429790798877458; */
  /* params.u_max[1] = 1.855058551624045; */
  /* params.u_max[2] = 0.44067266309662534; */
  /* params.u_max[3] = 0.5038606051092378; */
}
