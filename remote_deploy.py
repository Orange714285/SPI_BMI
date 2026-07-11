import pty
import os
import sys
import time

def run_with_password(cmd, password):
    master, slave = pty.openpty()
    pid = os.fork()
    if pid == 0:
        os.dup2(slave, 0)
        os.dup2(slave, 1)
        os.dup2(slave, 2)
        os.close(master)
        os.close(slave)
        os.execvp(cmd[0], cmd)
    else:
        os.close(slave)
        buffer = b""
        password_sent = False
        start_time = time.time()
        while True:
            try:
                r = os.read(master, 1024)
                if not r:
                    break
                sys.stdout.buffer.write(r)
                sys.stdout.flush()
                buffer += r
                if b"password:" in buffer.lower() and not password_sent:
                    time.sleep(0.5)
                    os.write(master, (password + "\n").encode())
                    password_sent = True
                    buffer = b""
            except OSError:
                break
            if time.time() - start_time > 15:
                print("\n[ERROR] Command timed out")
                break
        
        _, status = os.waitpid(pid, 0)
        return status

print("\nUploading check_frequency.py to Raspberry Pi...")
scp_cmd = ["scp", "check_frequency.py", "rp@10.42.0.219:~/check_frequency.py"]
run_with_password(scp_cmd, "ubuntu")

print("\nRunning check_frequency.py on Raspberry Pi...")
ssh_cmd = ["ssh", "rp@10.42.0.219", "python3 ~/check_frequency.py"]
run_with_password(ssh_cmd, "ubuntu")
