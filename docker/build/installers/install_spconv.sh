# 直接使用脚本安装即可
export CUMM_VERSION=0.5.3
export SPCONV_VERSION=2.3.8
ansible-playbook autoware.dev_env.install_spconv.yaml -e cumm_version=${CUMM_VERSION} -e spconv_version=${SPCONV_VERSION} --ask-become-pass