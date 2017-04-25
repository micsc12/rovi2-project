
add these lines to .bash_aliases file:

# aliases for initialization of RoVi workcells
alias initWC2='. /<your path from root to file>/InitWorkcell2.sh'
alias initWC3='. /<your path from root to file>/InitWorkcell3.sh'

Change ROS_IP to your value in all files.
Check that you have all files needed!!! An now run in every terminal, which needs to be initialized command:
    initWC2 or iniWC3


OR:
to export adresses using InitWorkcellX.sh run (from directory containing this file):
 . ./InitWorkcellX.sh
 
first <.> is IMPORTANT


