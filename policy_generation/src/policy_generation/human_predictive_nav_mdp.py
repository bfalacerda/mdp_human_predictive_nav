from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef


class HumanPredictiveNavMdp(Mdp):
    def __init__(self, human_mc_file):
        stream=open(human_mc_file,'r')
        for line in stream:
            print line
        stream.close()