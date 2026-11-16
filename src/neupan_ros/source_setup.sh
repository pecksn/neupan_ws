
# Module contract: Legacy interactive shell-configuration helper.
# This file appends a workspace source command to a user shell startup file; it
# is not a read-only environment probe and must not be executed by verification.
# Repeated invocation can add repeated entries. Preserve existing behavior here;
# normal container startup instructions are maintained in the operator guide.
if [ -n "$($SHELL -c 'echo $ZSH_VERSION')" ]; then
   echo 'source ~/neupan_ws/devel/setup.zsh' >> ~/.zshrc
   zsh -c 'source ~/.zshrc'
elif [ -n "$($SHELL -c 'echo $BASH_VERSION')" ]; then
   echo 'source ~/neupan_ws/devel/setup.bash' >> ~/.bashrc
   bash -c 'source ~/.bashrc'
fi
