#!/bin/bash
# One-time root install for the E34 mitigation.
#
# The IgH master kernel thread (EtherCAT-OP) spawns SCHED_NORMAL nice 0
# (master.c:1681) and the master lock is a plain semaphore with no priority
# inheritance (master.h:198). Under board load the thread can sit runnable
# while holding that lock and every 1 kHz ioctl of the app stalls behind it
# (up to 66% cycle loss observed, merge.md §9.5 E34).
#
# This installs:
#   /usr/local/sbin/ecat-op-fifo   - fixed-command wrapper: lift EtherCAT-OP
#                                    to SCHED_FIFO 40 (below TASK1's 95)
#   /etc/sudoers.d/ecat-op-fifo    - lets user 'ubuntu' run ONLY that wrapper
#                                    without a password, so run.sh can call it
#                                    automatically after every app start
#
# Run once:  sudo bash tools/install_ecat_op_fifo.sh
set -e
if [ "$(id -u)" != 0 ]; then
    echo "run with sudo: sudo bash $0" >&2
    exit 1
fi

install -m 0755 /dev/stdin /usr/local/sbin/ecat-op-fifo <<'EOF'
#!/bin/sh
# E34: lift the IgH EtherCAT-OP kernel thread to SCHED_FIFO 40.
# Fixed command on purpose - referenced by a NOPASSWD sudoers rule.
pid=$(pgrep -x EtherCAT-OP) || exit 1
exec chrt -f -p 40 "$pid"
EOF

cat > /etc/sudoers.d/ecat-op-fifo <<'EOF'
ubuntu ALL=(root) NOPASSWD: /usr/local/sbin/ecat-op-fifo
EOF
chmod 0440 /etc/sudoers.d/ecat-op-fifo
visudo -cf /etc/sudoers.d/ecat-op-fifo

echo "installed: /usr/local/sbin/ecat-op-fifo + /etc/sudoers.d/ecat-op-fifo"
echo "run.sh will now lift EtherCAT-OP to FIFO 40 automatically on every start."
