/* Produced by CVXGEN, 2013-12-03 15:01:24 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.kappa[0] = 1.101595805149151;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.mass[0] = 1.7081478226181048;
  params.mass[7] = 0;
  params.mass[14] = 0;
  params.mass[21] = 0;
  params.mass[28] = 0;
  params.mass[35] = 0;
  params.mass[42] = 0;
  params.mass[1] = 0;
  params.mass[8] = 1.2909047389129444;
  params.mass[15] = 0;
  params.mass[22] = 0;
  params.mass[29] = 0;
  params.mass[36] = 0;
  params.mass[43] = 0;
  params.mass[2] = 0;
  params.mass[9] = 0;
  params.mass[16] = 1.510827605197663;
  params.mass[23] = 0;
  params.mass[30] = 0;
  params.mass[37] = 0;
  params.mass[44] = 0;
  params.mass[3] = 0;
  params.mass[10] = 0;
  params.mass[17] = 0;
  params.mass[24] = 1.8929469543476547;
  params.mass[31] = 0;
  params.mass[38] = 0;
  params.mass[45] = 0;
  params.mass[4] = 0;
  params.mass[11] = 0;
  params.mass[18] = 0;
  params.mass[25] = 0;
  params.mass[32] = 1.896293088933438;
  params.mass[39] = 0;
  params.mass[46] = 0;
  params.mass[5] = 0;
  params.mass[12] = 0;
  params.mass[19] = 0;
  params.mass[26] = 0;
  params.mass[33] = 0;
  params.mass[40] = 1.1255853104638363;
  params.mass[47] = 0;
  params.mass[6] = 0;
  params.mass[13] = 0;
  params.mass[20] = 0;
  params.mass[27] = 0;
  params.mass[34] = 0;
  params.mass[41] = 0;
  params.mass[48] = 1.2072428781381868;
  params.tau_max_delta_t[0] = -1.7941311867966805;
  params.tau_max_delta_t[1] = -0.23676062539745413;
  params.tau_max_delta_t[2] = -1.8804951564857322;
  params.tau_max_delta_t[3] = -0.17266710242115568;
  params.tau_max_delta_t[4] = 0.596576190459043;
  params.tau_max_delta_t[5] = -0.8860508694080989;
  params.tau_max_delta_t[6] = 0.7050196079205251;
  params.alpha[0] = 1.1817256348327017;
  params.posture_weight[0] = 0.04796376475433073;
  params.delta_q_des[0] = 0.23541635196352795;
  params.delta_q_des[1] = -0.9629902123701384;
  params.delta_q_des[2] = -0.3395952119597214;
  params.delta_q_des[3] = -0.865899672914725;
  params.delta_q_des[4] = 0.7725516732519853;
  params.delta_q_des[5] = -0.23818512931704205;
  params.delta_q_des[6] = -1.372529046100147;
  params.q_0[0] = 0.17859607212737894;
  params.q_0[1] = 1.1212590580454682;
  params.q_0[2] = -0.774545870495281;
  params.q_0[3] = -1.1121684642712744;
  params.q_0[4] = -0.44811496977740495;
  params.q_0[5] = 1.7455345994417217;
  params.q_0[6] = 1.9039816898917352;
  params.zeta[0] = 1.3447673518256273;
  params.xyz_weight[0] = 1.8056682170767961;
  params.delta_x_des[0] = 1.383003485172717;
  params.delta_x_des[1] = -0.48802383468444344;
  params.delta_x_des[2] = -1.631131964513103;
  params.J[0] = 0.6136436100941447;
  params.J[1] = 0.2313630495538037;
  params.J[2] = -0.5537409477496875;
  params.J[3] = -1.0997819806406723;
  params.J[4] = -0.3739203344950055;
  params.J[5] = -0.12423900520332376;
  params.J[6] = -0.923057686995755;
  params.J[7] = -0.8328289030982696;
  params.J[8] = -0.16925440270808823;
  params.J[9] = 1.442135651787706;
  params.J[10] = 0.34501161787128565;
  params.J[11] = -0.8660485502711608;
  params.J[12] = -0.8880899735055947;
  params.J[13] = -0.1815116979122129;
  params.J[14] = -1.17835862158005;
  params.J[15] = -1.1944851558277074;
  params.J[16] = 0.05614023926976763;
  params.J[17] = -1.6510825248767813;
  params.J[18] = -0.06565787059365391;
  params.J[19] = -0.5512951504486665;
  params.J[20] = 0.8307464872626844;
  params.mu[0] = 1.4934924462040091;
  params.beta[0] = 1.3821858437115286;
  params.n_K_J_all[0] = 0.7567216550196565;
  params.n_K_J_all[1] = -0.5055995034042868;
  params.n_K_J_all[2] = 0.6725392189410702;
  params.n_K_J_all[3] = -0.6406053441727284;
  params.n_K_J_all[4] = 0.29117547947550015;
  params.n_K_J_all[5] = -0.6967713677405021;
  params.n_K_J_all[6] = -0.21941980294587182;
  params.n_K_J_all[7] = -1.753884276680243;
  params.n_K_J_all[8] = -1.0292983112626475;
  params.n_K_J_all[9] = 1.8864104246942706;
  params.n_K_J_all[10] = -1.077663182579704;
  params.n_K_J_all[11] = 0.7659100437893209;
  params.n_K_J_all[12] = 0.6019074328549583;
  params.n_K_J_all[13] = 0.8957565577499285;
  params.n_K_J_all[14] = -0.09964555746227477;
  params.n_K_J_all[15] = 0.38665509840745127;
  params.n_K_J_all[16] = -1.7321223042686946;
  params.n_K_J_all[17] = -1.7097514487110663;
  params.n_K_J_all[18] = -1.2040958948116867;
  params.n_K_J_all[19] = -1.3925560119658358;
  params.n_K_J_all[20] = -1.5995826216742213;
  params.n_K_J_all[21] = -1.4828245415645833;
  params.n_K_J_all[22] = 0.21311092723061398;
  params.n_K_J_all[23] = -1.248740700304487;
  params.n_K_J_all[24] = 1.808404972124833;
  params.n_K_J_all[25] = 0.7264471152297065;
  params.n_K_J_all[26] = 0.16407869343908477;
  params.n_K_J_all[27] = 0.8287224032315907;
  params.n_K_J_all[28] = -0.9444533161899464;
  params.n_K_J_all[29] = 1.7069027370149112;
  params.n_K_J_all[30] = 1.3567722311998827;
  params.n_K_J_all[31] = 0.9052779937121489;
  params.n_K_J_all[32] = -0.07904017565835986;
  params.n_K_J_all[33] = 1.3684127435065871;
  params.n_K_J_all[34] = 0.979009293697437;
  params.n_K_J_all[35] = 0.6413036255984501;
  params.n_K_J_all[36] = 1.6559010680237511;
  params.n_K_J_all[37] = 0.5346622551502991;
  params.n_K_J_all[38] = -0.5362376605895625;
  params.n_K_J_all[39] = 0.2113782926017822;
  params.n_K_J_all[40] = -1.2144776931994525;
  params.n_K_J_all[41] = -1.2317108144255875;
  params.n_K_J_all[42] = 0.9026784957312834;
  params.n_K_J_all[43] = 1.1397468137245244;
  params.n_K_J_all[44] = 1.8883934547350631;
  params.n_K_J_all[45] = 1.4038856681660068;
  params.n_K_J_all[46] = 0.17437730638329096;
  params.n_K_J_all[47] = -1.6408365219077408;
  params.n_K_J_all[48] = -0.04450702153554875;
  params.n_K_J_all[49] = 1.7117453902485025;
  params.n_K_J_all[50] = 1.1504727980139053;
  params.n_K_J_all[51] = -0.05962309578364744;
  params.n_K_J_all[52] = -0.1788825540764547;
  params.n_K_J_all[53] = -1.1280569263625857;
  params.n_K_J_all[54] = -1.2911464767927057;
  params.n_K_J_all[55] = -1.7055053231225696;
  params.n_K_J_all[56] = 1.56957275034837;
  params.n_K_J_all[57] = 0.5607064675962357;
  params.n_K_J_all[58] = -1.4266707301147146;
  params.n_K_J_all[59] = -0.3434923211351708;
  params.n_K_J_all[60] = -1.8035643024085055;
  params.n_K_J_all[61] = -1.1625066019105454;
  params.n_K_J_all[62] = 0.9228324965161532;
  params.n_K_J_all[63] = 0.6044910817663975;
  params.n_K_J_all[64] = -0.0840868104920891;
  params.n_K_J_all[65] = -0.900877978017443;
  params.n_K_J_all[66] = 0.608892500264739;
  params.n_K_J_all[67] = 1.8257980452695217;
  params.n_K_J_all[68] = -0.25791777529922877;
  params.n_K_J_all[69] = -1.7194699796493191;
  params.delta_f_max[0] = -1.7690740487081298;
  params.delta_f_max[1] = -1.6685159248097703;
  params.delta_f_max[2] = 1.8388287490128845;
  params.delta_f_max[3] = 0.16304334474597537;
  params.delta_f_max[4] = 1.3498497306788897;
  params.delta_f_max[5] = -1.3198658230514613;
  params.delta_f_max[6] = -0.9586197090843394;
  params.delta_f_max[7] = 0.7679100474913709;
  params.delta_f_max[8] = 1.5822813125679343;
  params.delta_f_max[9] = -0.6372460621593619;
  params.A_tl[0] = -1.741307208038867;
  params.A_tl[1] = 1.456478677642575;
  params.A_tl[2] = -0.8365102166820959;
  params.A_tl[3] = 0.9643296255982503;
  params.A_tl[4] = -1.367865381194024;
  params.A_tl[5] = 0.7798537405635035;
  params.A_tl[6] = 1.3656784761245926;
  params.A_tl[7] = 0.9086083149868371;
  params.A_tl[8] = -0.5635699005460344;
  params.A_tl[9] = 0.9067590059607915;
  params.A_tl[10] = -1.4421315032701587;
  params.A_tl[11] = -0.7447235390671119;
  params.A_tl[12] = -0.32166897326822186;
  params.A_tl[13] = 1.5088481557772684;
  params.A_tl[14] = -1.385039165715428;
  params.A_tl[15] = 1.5204991609972622;
  params.A_tl[16] = 1.1958572768832156;
  params.A_tl[17] = 1.8864971883119228;
  params.A_tl[18] = -0.5291880667861584;
  params.A_tl[19] = -1.1802409243688836;
  params.A_tl[20] = -1.037718718661604;
  params.A_tl[21] = 1.3114512056856835;
  params.A_tl[22] = 1.8609125943756615;
  params.A_tl[23] = 0.7952399935216938;
  params.A_tl[24] = -0.07001183290468038;
  params.A_tl[25] = -0.8518009412754686;
  params.A_tl[26] = 1.3347515373726386;
  params.A_tl[27] = 1.4887180335977037;
  params.A_tl[28] = -1.6314736327976336;
  params.A_tl[29] = -1.1362021159208933;
  params.A_tl[30] = 1.327044361831466;
  params.A_tl[31] = 1.3932155883179842;
  params.A_tl[32] = -0.7413880049440107;
  params.A_tl[33] = -0.8828216126125747;
  params.A_tl[34] = -0.27673991192616;
  params.A_tl[35] = 0.15778600105866714;
  params.A_tl[36] = -1.6177327399735457;
  params.A_tl[37] = 1.3476485548544606;
  params.A_tl[38] = 0.13893948140528378;
  params.A_tl[39] = 1.0998712601636944;
  params.A_tl[40] = -1.0766549376946926;
  params.A_tl[41] = 1.8611734044254629;
  params.A_tl[42] = 1.0041092292735172;
  params.A_tl[43] = -0.6276245424321543;
  params.A_tl[44] = 1.794110587839819;
  params.A_tl[45] = 0.8020471158650913;
  params.A_tl[46] = 1.362244341944948;
  params.A_tl[47] = -1.8180107765765245;
  params.A_tl[48] = -1.7774338357932473;
  params.qd_0[0] = 0.9709490941985153;
  params.qd_0[1] = -0.7812542682064318;
  params.qd_0[2] = 0.0671374633729811;
  params.qd_0[3] = -1.374950305314906;
  params.qd_0[4] = 1.9118096386279388;
  params.qd_0[5] = 0.011004190697677885;
  params.qd_0[6] = 1.3160043138989015;
  params.A_tr[0] = -1.7038488148800144;
  params.A_tr[1] = -0.08433819112864738;
  params.A_tr[2] = -1.7508820783768964;
  params.A_tr[3] = 1.536965724350949;
  params.A_tr[4] = -0.21675928514816478;
  params.A_tr[5] = -1.725800326952653;
  params.A_tr[6] = -1.6940148707361717;
  params.A_tr[7] = 0.15517063201268;
  params.A_tr[8] = -1.697734381979077;
  params.A_tr[9] = -1.264910727950229;
  params.A_tr[10] = -0.2545716633339441;
  params.A_tr[11] = -0.008868675926170244;
  params.A_tr[12] = 0.3332476609670296;
  params.A_tr[13] = 0.48205072561962936;
  params.A_tr[14] = -0.5087540014293261;
  params.A_tr[15] = 0.4749463319223195;
  params.A_tr[16] = -1.371021366459455;
  params.A_tr[17] = -0.8979660982652256;
  params.A_tr[18] = 1.194873082385242;
  params.A_tr[19] = -1.3876427970939353;
  params.A_tr[20] = -1.106708108457053;
  params.A_tr[21] = -1.0280872812241797;
  params.A_tr[22] = -0.08197078070773234;
  params.A_tr[23] = -1.9970179118324083;
  params.A_tr[24] = -1.878754557910134;
  params.A_tr[25] = -0.15380739340877803;
  params.A_tr[26] = -1.349917260533923;
  params.A_tr[27] = 0.7180072150931407;
  params.A_tr[28] = 1.1808183487065538;
  params.A_tr[29] = 0.31265343495084075;
  params.A_tr[30] = 0.7790599086928229;
  params.A_tr[31] = -0.4361679370644853;
  params.A_tr[32] = -1.8148151880282066;
  params.A_tr[33] = -0.24231386948140266;
  params.A_tr[34] = -0.5120787511622411;
  params.A_tr[35] = 0.3880129688013203;
  params.A_tr[36] = -1.4631273212038676;
  params.A_tr[37] = -1.0891484131126563;
  params.A_tr[38] = 1.2591296661091191;
  params.A_tr[39] = -0.9426978934391474;
  params.A_tr[40] = -0.358719180371347;
  params.A_tr[41] = 1.7438887059831263;
  params.A_tr[42] = -0.8977901479165817;
  params.A_tr[43] = -1.4188401645857445;
  params.A_tr[44] = 0.8080805173258092;
  params.A_tr[45] = 0.2682662017650985;
  params.A_tr[46] = 0.44637534218638786;
  params.A_tr[47] = -1.8318765960257055;
  params.A_tr[48] = -0.3309324209710929;
  params.B_t1[0] = -1.9829342633313622;
  params.B_t1[1] = -1.013858124556442;
  params.B_t1[2] = 0.8242247343360254;
  params.B_t1[3] = -1.753837136317201;
  params.B_t1[4] = -0.8212260055868805;
  params.B_t1[5] = 1.9524510112487126;
  params.B_t1[6] = 1.884888920907902;
  params.B_t1[7] = -0.0726144452811801;
  params.B_t1[8] = 0.9427735461129836;
  params.B_t1[9] = 0.5306230967445558;
  params.B_t1[10] = -0.1372277142250531;
  params.B_t1[11] = 1.4282657305652786;
  params.B_t1[12] = -1.309926991335284;
  params.B_t1[13] = 1.3137276889764422;
  params.B_t1[14] = -1.8317219061667278;
  params.B_t1[15] = 1.4678147672511939;
  params.B_t1[16] = 0.703986349872991;
  params.B_t1[17] = -0.2163435603565258;
  params.B_t1[18] = 0.6862809905371079;
  params.B_t1[19] = -0.15852598444303245;
  params.B_t1[20] = 1.1200128895143409;
  params.B_t1[21] = -1.5462236645435308;
  params.B_t1[22] = 0.0326297153944215;
  params.B_t1[23] = 1.4859581597754916;
  params.B_t1[24] = 1.71011710324809;
  params.B_t1[25] = -1.1186546738067493;
  params.B_t1[26] = -0.9922787897815244;
  params.B_t1[27] = 1.6160498864359547;
  params.B_t1[28] = -0.6179306451394861;
  params.B_t1[29] = -1.7725097038051376;
  params.B_t1[30] = 0.8595466884481313;
  params.B_t1[31] = -0.3423245633865686;
  params.B_t1[32] = 0.9412967499805762;
  params.B_t1[33] = -0.09163346622652258;
  params.B_t1[34] = 0.002262217745727657;
  params.B_t1[35] = -0.3297523583656421;
  params.B_t1[36] = -0.8380604158593941;
  params.B_t1[37] = 1.6028434695494038;
  params.B_t1[38] = 0.675150311940429;
  params.B_t1[39] = 1.1553293733718686;
  params.B_t1[40] = 1.5829581243724693;
  params.B_t1[41] = -0.9992442304425597;
  params.B_t1[42] = 1.6792824558896897;
  params.B_t1[43] = 1.4504203490342324;
  params.B_t1[44] = 0.02434104849994556;
  params.B_t1[45] = 0.27160869657612263;
  params.B_t1[46] = -1.5402710478528858;
  params.B_t1[47] = 1.0484633622310744;
  params.B_t1[48] = -1.3070999712627054;
  params.q_des_cur_0[0] = 0.13534416402363814;
  params.q_des_cur_0[1] = -1.4942507790851232;
  params.q_des_cur_0[2] = -1.708331625671371;
  params.q_des_cur_0[3] = 0.436109775042258;
  params.q_des_cur_0[4] = -0.03518748153727991;
  params.q_des_cur_0[5] = 0.6992397389570906;
  params.q_des_cur_0[6] = 1.1634167322171374;
  params.B_t2[0] = 1.9307499705822648;
  params.B_t2[1] = -1.6636772756932747;
  params.B_t2[2] = 0.5248484497343218;
  params.B_t2[3] = 0.30789958152579144;
  params.B_t2[4] = 0.602568707166812;
  params.B_t2[5] = 0.17271781925751872;
  params.B_t2[6] = 0.2294695501208066;
  params.B_t2[7] = 1.4742185345619543;
  params.B_t2[8] = -0.1919535345136989;
  params.B_t2[9] = 0.13990231452144553;
  params.B_t2[10] = 0.7638548150610602;
  params.B_t2[11] = -1.6420200344195646;
  params.B_t2[12] = -0.27229872445076087;
  params.B_t2[13] = -1.5914631171820468;
  params.B_t2[14] = -1.4487604283558668;
  params.B_t2[15] = -1.991497766136364;
  params.B_t2[16] = -1.1611742553535152;
  params.B_t2[17] = -1.133450950247063;
  params.B_t2[18] = 0.06497792493777155;
  params.B_t2[19] = 0.28083295396097263;
  params.B_t2[20] = 1.2958447220129887;
  params.B_t2[21] = -0.05315524470737154;
  params.B_t2[22] = 1.5658183956871667;
  params.B_t2[23] = -0.41975684089933685;
  params.B_t2[24] = 0.97844578833777;
  params.B_t2[25] = 0.2110290496695293;
  params.B_t2[26] = 0.4953003430893044;
  params.B_t2[27] = -0.9184320124667495;
  params.B_t2[28] = 1.750380031759156;
  params.B_t2[29] = 1.0786188614315915;
  params.B_t2[30] = -1.4176198837203735;
  params.B_t2[31] = 0.149737479778294;
  params.B_t2[32] = 1.9831452222223418;
  params.B_t2[33] = -1.8037746699794734;
  params.B_t2[34] = -0.7887206483295461;
  params.B_t2[35] = 0.9632534854086652;
  params.B_t2[36] = -1.8425542093895406;
  params.B_t2[37] = 0.986684363969033;
  params.B_t2[38] = 0.2936851199350441;
  params.B_t2[39] = 0.9268227022482662;
  params.B_t2[40] = 0.20333038350653299;
  params.B_t2[41] = 1.7576139132046351;
  params.B_t2[42] = -0.614393188398918;
  params.B_t2[43] = 0.297877839744912;
  params.B_t2[44] = -1.796880083990895;
  params.B_t2[45] = 0.21373133661742738;
  params.B_t2[46] = -0.32242822540825156;
  params.B_t2[47] = 1.9326471511608059;
  params.B_t2[48] = 1.7824292753481785;
  params.tau_cont_sum_0[0] = -1.4468823405675986;
  params.tau_cont_sum_0[1] = -1.8335374338761512;
  params.tau_cont_sum_0[2] = -1.5172997317243713;
  params.tau_cont_sum_0[3] = -1.229012129120719;
  params.tau_cont_sum_0[4] = 0.9046719772422094;
  params.tau_cont_sum_0[5] = 0.17591181415489432;
  params.tau_cont_sum_0[6] = 0.13970133814112584;
  params.B_t3[0] = -0.14185208214985234;
  params.B_t3[1] = -1.9732231264739348;
  params.B_t3[2] = -0.4301123458221334;
  params.B_t3[3] = 1.9957537650387742;
  params.B_t3[4] = 1.2811648216477893;
  params.B_t3[5] = 0.2914428437588219;
  params.B_t3[6] = -1.214148157218884;
  params.B_t3[7] = 1.6818776980374155;
  params.B_t3[8] = -0.30341101038214635;
  params.B_t3[9] = 0.47730909231793106;
  params.B_t3[10] = -1.187569373035299;
  params.B_t3[11] = -0.6877370247915531;
  params.B_t3[12] = -0.6201861482616171;
  params.B_t3[13] = -0.4209925183921568;
  params.B_t3[14] = -1.9110724537712471;
  params.B_t3[15] = 0.6413882087807936;
  params.B_t3[16] = -1.3200399280087032;
  params.B_t3[17] = 0.41320105301312626;
  params.B_t3[18] = 0.4783213861392275;
  params.B_t3[19] = 0.7916189857293743;
  params.B_t3[20] = -0.8322752558146558;
  params.B_t3[21] = -0.8318720537426154;
  params.B_t3[22] = 1.0221179076113445;
  params.B_t3[23] = -0.4471032189262627;
  params.B_t3[24] = -1.3901469561676985;
  params.B_t3[25] = 1.6210596051208572;
  params.B_t3[26] = -1.9476687601912737;
  params.B_t3[27] = 1.5459376306231292;
  params.B_t3[28] = -0.830972896191656;
  params.B_t3[29] = -0.47269983955176276;
  params.B_t3[30] = 1.913620609584223;
  params.B_t3[31] = -0.25329703423935124;
  params.B_t3[32] = 0.8635279149674653;
  params.B_t3[33] = -0.35046893227111564;
  params.B_t3[34] = 1.6541432486772365;
  params.B_t3[35] = 0.8779619968413503;
  params.B_t3[36] = -0.07723284625844862;
  params.B_t3[37] = -1.6631134040635196;
  params.B_t3[38] = -0.54546452868516;
  params.B_t3[39] = -0.03757319061095998;
  params.B_t3[40] = -0.864543266194465;
  params.B_t3[41] = 0.13856203767859343;
  params.B_t3[42] = -1.1613957272733684;
  params.B_t3[43] = -0.022681697832835024;
  params.B_t3[44] = 0.11202078062843634;
  params.B_t3[45] = 0.6934385624164641;
  params.B_t3[46] = 0.9814633803279791;
  params.B_t3[47] = 0.9198949681022897;
  params.B_t3[48] = -0.3035363988458051;
  params.A_bl[0] = -0.1761906755724203;
  params.A_bl[1] = 1.4940284058791686;
  params.A_bl[2] = -0.5488483097174393;
  params.A_bl[3] = 0.9521313238305416;
  params.A_bl[4] = 1.9762689267600413;
  params.A_bl[5] = 1.6992335341478482;
  params.A_bl[6] = 0.1969474711697119;
  params.A_bl[7] = -0.7795544525014559;
  params.A_bl[8] = 0.4892505434034007;
  params.A_bl[9] = 0.7372066729248594;
  params.A_bl[10] = 0.10784901966517557;
  params.A_bl[11] = -0.6340934767066218;
  params.A_bl[12] = -0.17829371464242083;
  params.A_bl[13] = -1.6728370279392784;
  params.A_bl[14] = -0.8348711800042916;
  params.A_bl[15] = -1.4204129800590897;
  params.A_bl[16] = 0.6659229232859376;
  params.A_bl[17] = 1.8369365661533168;
  params.A_bl[18] = -1.371061267737546;
  params.A_bl[19] = -1.8868237125934915;
  params.A_bl[20] = 0.9654286768651104;
  params.A_bl[21] = -0.5833420409292005;
  params.A_bl[22] = 0.02386510653728502;
  params.A_bl[23] = -1.7558076992858345;
  params.A_bl[24] = -1.2889402130475411;
  params.A_bl[25] = 0.7820251677632606;
  params.A_bl[26] = 0.4208424784688227;
  params.A_bl[27] = 1.4136448896755982;
  params.A_bl[28] = 1.8516928541530757;
  params.A_bl[29] = -0.5615396035790421;
  params.A_bl[30] = 0.4809940266433177;
  params.A_bl[31] = -0.20929035114697303;
  params.A_bl[32] = 0.022387850798402553;
  params.A_bl[33] = -0.43399296564115764;
  params.A_bl[34] = 1.9095769077945013;
  params.A_bl[35] = 0.4945512698336847;
  params.A_bl[36] = -1.4324582900293557;
  params.A_bl[37] = 0.790913765746676;
  params.A_bl[38] = 1.8630250293383734;
  params.A_bl[39] = 1.5793975466121069;
  params.A_bl[40] = 0.2320163334712646;
  params.A_bl[41] = -1.9411408650055968;
  params.A_bl[42] = 1.2221853270725478;
  params.A_bl[43] = 1.7274453600045607;
  params.A_bl[44] = 0.9357159281665783;
  params.A_bl[45] = -0.2841874908331623;
  params.A_bl[46] = -0.4766355664552626;
  params.A_bl[47] = 0.9784190546201912;
  params.A_bl[48] = -1.5685956114005477;
  params.A_br[0] = 1.1387833891036;
  params.A_br[1] = -0.004779126480003892;
  params.A_br[2] = -1.7195239474925414;
  params.A_br[3] = 1.2921808565147272;
  params.A_br[4] = -0.43317009071966606;
  params.A_br[5] = -1.572940257279357;
  params.A_br[6] = -1.3048062231674988;
  params.A_br[7] = 1.4377304631579175;
  params.A_br[8] = -1.3090328020145874;
  params.A_br[9] = 1.1370018620707785;
  params.A_br[10] = 1.2164644012668289;
  params.A_br[11] = -1.6539274174499985;
  params.A_br[12] = -0.25845368809725544;
  params.A_br[13] = 1.1486358936399745;
  params.A_br[14] = -0.03975647517318137;
  params.A_br[15] = 1.4640632749164326;
  params.A_br[16] = -0.48111499989733186;
  params.A_br[17] = 0.5132576752843594;
  params.A_br[18] = -1.1459189400462249;
  params.A_br[19] = 1.3690255364554855;
  params.A_br[20] = 1.3574291456003253;
  params.A_br[21] = 0.26333733823037253;
  params.A_br[22] = -0.7076462135286032;
  params.A_br[23] = -0.6097272363453645;
  params.A_br[24] = 0.37873096815108465;
  params.A_br[25] = -1.4863636934585411;
  params.A_br[26] = 0.04189135833804869;
  params.A_br[27] = -0.8182949160834703;
  params.A_br[28] = -0.6336865828985854;
  params.A_br[29] = -0.7126437991119396;
  params.A_br[30] = 1.3381487344587226;
  params.A_br[31] = -1.2979975504895949;
  params.A_br[32] = -1.0542097271412714;
  params.A_br[33] = -1.3421003125955435;
  params.A_br[34] = -1.9395969070507038;
  params.A_br[35] = -0.29758108058547306;
  params.A_br[36] = 1.3757899684264032;
  params.A_br[37] = 1.6109970296148042;
  params.A_br[38] = -0.050537352418498216;
  params.A_br[39] = -0.3144945653528741;
  params.A_br[40] = 1.4726689240031474;
  params.A_br[41] = 0.11397910876468265;
  params.A_br[42] = 0.19466869962815858;
  params.A_br[43] = 0.5972476722406035;
  params.A_br[44] = -1.6815490772221828;
  params.A_br[45] = 1.3540223072599735;
  params.A_br[46] = -1.577027832358222;
  params.A_br[47] = 0.12928618615237353;
  params.A_br[48] = 1.704038169667271;
  params.B_b1[0] = 0.19482725189070793;
  params.B_b1[1] = -0.6311686254597215;
  params.B_b1[2] = 0.9065234706582928;
  params.B_b1[3] = 1.604058201281767;
  params.B_b1[4] = 0.4649414640474294;
  params.B_b1[5] = -1.7764554290993346;
  params.B_b1[6] = 1.5152343936830337;
  params.B_b1[7] = -1.9280901945449935;
  params.B_b1[8] = 0.7129569482366098;
  params.B_b1[9] = 1.6001840923928201;
  params.B_b1[10] = -1.3702177446733126;
  params.B_b1[11] = 0.11266051920028186;
  params.B_b1[12] = 0.8202183589903962;
  params.B_b1[13] = -1.297953481011172;
  params.B_b1[14] = -1.0192096617939002;
  params.B_b1[15] = -1.7337200441949867;
  params.B_b1[16] = -1.3639899659742465;
  params.B_b1[17] = -1.5273517222086332;
  params.B_b1[18] = -0.8374302703303731;
  params.B_b1[19] = 1.00229367551592;
  params.B_b1[20] = 0.7747378843920099;
  params.B_b1[21] = 1.0504096866871468;
  params.B_b1[22] = 0.638655773812761;
  params.B_b1[23] = 1.176936790033046;
  params.B_b1[24] = -1.4041747524796162;
  params.B_b1[25] = 0.21725437512222667;
  params.B_b1[26] = -1.9141609882936188;
  params.B_b1[27] = -0.03334441105363828;
  params.B_b1[28] = 1.3736673884387467;
  params.B_b1[29] = -0.11085150689269163;
  params.B_b1[30] = -0.8176560931958075;
  params.B_b1[31] = -0.9013799953302866;
  params.B_b1[32] = -0.42583422050124753;
  params.B_b1[33] = 1.6552920005330618;
  params.B_b1[34] = 1.8971842560697287;
  params.B_b1[35] = 0.9935321777966784;
  params.B_b1[36] = 1.9500402929402196;
  params.B_b1[37] = 1.0489535977170181;
  params.B_b1[38] = -0.8630392743714372;
  params.B_b1[39] = -0.25967183338596733;
  params.B_b1[40] = 0.8925966402843359;
  params.B_b1[41] = 0.8373600738876834;
  params.B_b1[42] = 0.7125001994938436;
  params.B_b1[43] = -0.048447588572545275;
  params.B_b1[44] = -1.4274714856193604;
  params.B_b1[45] = 1.8385542904833923;
  params.B_b1[46] = -1.1195070325474288;
  params.B_b1[47] = 1.9175373793884956;
  params.B_b1[48] = -1.49030500627704;
  params.B_b2[0] = 1.9213425364706396;
  params.B_b2[1] = -0.49553546476315047;
  params.B_b2[2] = 1.2437464435895134;
  params.B_b2[3] = -1.970831509470568;
  params.B_b2[4] = -0.219996830259797;
  params.B_b2[5] = -1.0042329091607591;
  params.B_b2[6] = 0.7781008085794774;
  params.B_b2[7] = 0.65210699599452;
  params.B_b2[8] = -0.152326999732443;
  params.B_b2[9] = 0.8265434509993406;
  params.B_b2[10] = 1.9130464561754126;
  params.B_b2[11] = -1.6270096836882288;
  params.B_b2[12] = 0.2507042290048189;
  params.B_b2[13] = 0.7038441998600256;
  params.B_b2[14] = 0.5328743207925606;
  params.B_b2[15] = -0.9509907719589208;
  params.B_b2[16] = 1.499815178589135;
  params.B_b2[17] = -1.0178753663037017;
  params.B_b2[18] = 1.3798461831617561;
  params.B_b2[19] = -0.11708553759234386;
  params.B_b2[20] = -1.4276299186218124;
  params.B_b2[21] = 1.296518419303864;
  params.B_b2[22] = -1.6872707956138546;
  params.B_b2[23] = 1.1799585157870145;
  params.B_b2[24] = 0.4000488706320535;
  params.B_b2[25] = 1.506638004200894;
  params.B_b2[26] = 1.2128180682740366;
  params.B_b2[27] = -0.39211699471717854;
  params.B_b2[28] = -1.4592313874139302;
  params.B_b2[29] = -0.9352340128154211;
  params.B_b2[30] = -1.994709862977336;
  params.B_b2[31] = 0.6136129920637026;
  params.B_b2[32] = -1.6579503948780245;
  params.B_b2[33] = -1.2828456921062488;
  params.B_b2[34] = -1.0200938896697522;
  params.B_b2[35] = -0.3755900704115436;
  params.B_b2[36] = 0.747199791836243;
  params.B_b2[37] = -0.22212974213441683;
  params.B_b2[38] = 0.015082263441096089;
  params.B_b2[39] = -1.6271688108937168;
  params.B_b2[40] = -0.6472903955867526;
  params.B_b2[41] = -1.1733258209627806;
  params.B_b2[42] = 0.9565501943340924;
  params.B_b2[43] = -1.929389541307601;
  params.B_b2[44] = 0.4671837668673531;
  params.B_b2[45] = 0.7915477026785647;
  params.B_b2[46] = 0.018572068486599758;
  params.B_b2[47] = -1.8220899973808726;
  params.B_b2[48] = -0.995629851336445;
  params.B_b3[0] = -1.0486975119711213;
  params.B_b3[1] = -0.9289312699596386;
  params.B_b3[2] = -0.9472402942019333;
  params.B_b3[3] = 1.8908619466142156;
  params.B_b3[4] = 1.164645007668001;
  params.B_b3[5] = 1.5636429264767182;
  params.B_b3[6] = 0.8540115800503387;
  params.B_b3[7] = -0.6133530465568309;
  params.B_b3[8] = 1.7674136894457204;
  params.B_b3[9] = -0.06217940181271242;
  params.B_b3[10] = -1.2582602406204213;
  params.B_b3[11] = 0.9179968784775836;
  params.B_b3[12] = -0.9627796203753647;
  params.B_b3[13] = 1.2911416493727805;
  params.B_b3[14] = 0.9619156621267284;
  params.B_b3[15] = -0.8391987363014124;
  params.B_b3[16] = -0.16142857857315818;
  params.B_b3[17] = 0.8603892868304936;
  params.B_b3[18] = 0.672061858055037;
  params.B_b3[19] = 0.10631385676272265;
  params.B_b3[20] = -1.1434283104802896;
  params.B_b3[21] = -0.7024280087663541;
  params.B_b3[22] = 0.7791723379458899;
  params.B_b3[23] = -0.17206766925671246;
  params.B_b3[24] = 0.8714406054415362;
  params.B_b3[25] = 0.7364640800101268;
  params.B_b3[26] = -0.577393318625969;
  params.B_b3[27] = -1.603607371381821;
  params.B_b3[28] = 0.7231454736647596;
  params.B_b3[29] = -0.5776666119800344;
  params.B_b3[30] = 0.25985922282642804;
  params.B_b3[31] = -1.500019293846674;
  params.B_b3[32] = -1.41591503759888;
  params.B_b3[33] = -0.30464385789747794;
  params.B_b3[34] = 0.677515340905404;
  params.B_b3[35] = -1.5301412809058377;
  params.B_b3[36] = 1.097788736551506;
  params.B_b3[37] = 1.4054563154505293;
  params.B_b3[38] = 0.6904915185274869;
  params.B_b3[39] = 0.9984361169236493;
  params.B_b3[40] = -1.0460788838474921;
  params.B_b3[41] = -1.5989319614177124;
  params.B_b3[42] = -0.6834813660758638;
  params.B_b3[43] = -1.4978328637140224;
  params.B_b3[44] = -0.3340404173113156;
  params.B_b3[45] = 1.044497402438696;
  params.B_b3[46] = -0.875611719278079;
  params.B_b3[47] = 1.4233779191761733;
  params.B_b3[48] = -0.1880612910960302;
  params.q_min[0] = -1.3523791242997114;
  params.q_min[1] = 0.5691200673315562;
  params.q_min[2] = -0.24590364206081272;
  params.q_min[3] = -0.6790819241936314;
  params.q_min[4] = 0.06554105230580287;
  params.q_min[5] = 1.9642897275976492;
  params.q_min[6] = 1.0075323744403706;
  params.q_max[0] = -0.8257682212557649;
  params.q_max[1] = 1.7097592474973915;
  params.q_max[2] = -1.2633370473270409;
  params.q_max[3] = -0.3674317265957998;
  params.q_max[4] = -0.5096221670767425;
  params.q_max[5] = 1.9427867788797188;
  params.q_max[6] = -1.9819265272693376;
  params.torque_min[0] = 0.59706237040941;
  params.torque_min[1] = 0.03464508712113412;
  params.torque_min[2] = -1.3762622976910213;
  params.torque_min[3] = 0.5667125534930704;
  params.torque_min[4] = 1.2314327073557654;
  params.torque_min[5] = 1.067926609423298;
  params.torque_min[6] = -1.6894983403969057;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Kp[0] = 1.7950994324051095;
  params.Kp[7] = 0;
  params.Kp[14] = 0;
  params.Kp[21] = 0;
  params.Kp[28] = 0;
  params.Kp[35] = 0;
  params.Kp[42] = 0;
  params.Kp[1] = 0;
  params.Kp[8] = 1.3459072978954574;
  params.Kp[15] = 0;
  params.Kp[22] = 0;
  params.Kp[29] = 0;
  params.Kp[36] = 0;
  params.Kp[43] = 0;
  params.Kp[2] = 0;
  params.Kp[9] = 0;
  params.Kp[16] = 1.8525967635785667;
  params.Kp[23] = 0;
  params.Kp[30] = 0;
  params.Kp[37] = 0;
  params.Kp[44] = 0;
  params.Kp[3] = 0;
  params.Kp[10] = 0;
  params.Kp[17] = 0;
  params.Kp[24] = 1.5620952938330595;
  params.Kp[31] = 0;
  params.Kp[38] = 0;
  params.Kp[45] = 0;
  params.Kp[4] = 0;
  params.Kp[11] = 0;
  params.Kp[18] = 0;
  params.Kp[25] = 0;
  params.Kp[32] = 1.0799324049277437;
  params.Kp[39] = 0;
  params.Kp[46] = 0;
  params.Kp[5] = 0;
  params.Kp[12] = 0;
  params.Kp[19] = 0;
  params.Kp[26] = 0;
  params.Kp[33] = 0;
  params.Kp[40] = 1.7447506190695954;
  params.Kp[47] = 0;
  params.Kp[6] = 0;
  params.Kp[13] = 0;
  params.Kp[20] = 0;
  params.Kp[27] = 0;
  params.Kp[34] = 0;
  params.Kp[41] = 0;
  params.Kp[48] = 1.5635943825384881;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Kd[0] = 1.4514273849197905;
  params.Kd[7] = 0;
  params.Kd[14] = 0;
  params.Kd[21] = 0;
  params.Kd[28] = 0;
  params.Kd[35] = 0;
  params.Kd[42] = 0;
  params.Kd[1] = 0;
  params.Kd[8] = 1.6384584277378593;
  params.Kd[15] = 0;
  params.Kd[22] = 0;
  params.Kd[29] = 0;
  params.Kd[36] = 0;
  params.Kd[43] = 0;
  params.Kd[2] = 0;
  params.Kd[9] = 0;
  params.Kd[16] = 1.4437080645563034;
  params.Kd[23] = 0;
  params.Kd[30] = 0;
  params.Kd[37] = 0;
  params.Kd[44] = 0;
  params.Kd[3] = 0;
  params.Kd[10] = 0;
  params.Kd[17] = 0;
  params.Kd[24] = 1.0369253024633154;
  params.Kd[31] = 0;
  params.Kd[38] = 0;
  params.Kd[45] = 0;
  params.Kd[4] = 0;
  params.Kd[11] = 0;
  params.Kd[18] = 0;
  params.Kd[25] = 0;
  params.Kd[32] = 1.9127044824104826;
  params.Kd[39] = 0;
  params.Kd[46] = 0;
  params.Kd[5] = 0;
  params.Kd[12] = 0;
  params.Kd[19] = 0;
  params.Kd[26] = 0;
  params.Kd[33] = 0;
  params.Kd[40] = 1.2878053677966126;
  params.Kd[47] = 0;
  params.Kd[6] = 0;
  params.Kd[13] = 0;
  params.Kd[20] = 0;
  params.Kd[27] = 0;
  params.Kd[34] = 0;
  params.Kd[41] = 0;
  params.Kd[48] = 1.279344059595994;
  params.torque_max[0] = 0.09944832586730623;
  params.torque_max[1] = 1.4347906020137084;
  params.torque_max[2] = 1.8189082659711957;
  params.torque_max[3] = 0.9259085112867822;
  params.torque_max[4] = 1.6282484519835787;
  params.torque_max[5] = 0.657630418415067;
  params.torque_max[6] = 0.7694704654764088;
}
