"""Read-only trial recorder: start BEFORE enable/g; no publishers or services.

Run directly from the checkout in the sourced ROS2 environment. No rebuild.
Records status messages and the private filtered command, never camera images.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=int,default=60,choices=range(10,91),metavar='10..90')
    parser.add_argument('--output-dir',default=str(Path.home()/'husky_vision_ws/outputs/trial-trace'))
    args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import TwistStamped
    rclpy.init()
    node=Node('read_only_trial_trace')
    folder=Path(args.output_dir).expanduser();folder.mkdir(parents=True,exist_ok=True)
    stem=folder/('trial-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    started=time.monotonic();latest={};signatures={};counts=Counter();gate_reasons=Counter()
    seen_arm=False;disarmed_at=None;passed_gate=False;first_closure=None;max_command=0.
    subscriptions=[]
    with stem.with_suffix('.jsonl').open('x') as log:
        def status(name,message):
            nonlocal seen_arm,disarmed_at,passed_gate,first_closure
            try:
                data=json.loads(message.data)
                if not isinstance(data,dict):raise ValueError('non-object status')
            except (ValueError,TypeError):
                counts[name+'_invalid']+=1;return
            elapsed=round(time.monotonic()-started,3)
            latest[name]=data;counts[name]+=1
            log.write(json.dumps(dict(t_sec=elapsed,topic=name,data=data))+ '\n');log.flush()
            armed=latest.get('LINK',{}).get('receiver') or {}
            if name=='LINK':
                if (data.get('receiver') or {}).get('armed') is True:seen_arm=True
                elif seen_arm and disarmed_at is None:disarmed_at=time.monotonic()
            if name=='GATE' and armed.get('armed') is True:
                gate_reasons[data.get('reason','missing')]+=1
                if data.get('request_allowed') is True:passed_gate=True
                elif passed_gate and first_closure is None:
                    first_closure=dict(t_sec=elapsed,gate=data,filter=latest.get('FILTER'),
                                       link=latest.get('LINK'),ai=latest.get('AI'))
            summary={k:data[k] for k in ('reason','enabled','disable_reason','request_allowed','command_allowed') if k in data}
            if name=='LINK':
                receiver=data.get('receiver') or {}
                summary['gateway']={k:receiver.get(k) for k in ('armed','reason')}
            summary['nonzero']=abs(data.get('linear_x',0))>1e-6 or abs(data.get('angular_z',0))>1e-6
            key=json.dumps(summary,sort_keys=True)
            if signatures.get(name)!=key:
                print('%.3fs %s %s'%(elapsed,name,key),flush=True);signatures[name]=key
        def command(message):
            nonlocal max_command
            counts['FILTERED_COMMAND']+=1
            v,w=message.twist.linear.x,message.twist.angular.z
            max_command=max(max_command,abs(v))
            log.write(json.dumps(dict(t_sec=round(time.monotonic()-started,3),topic='FILTERED_COMMAND',
                linear_x=v,angular_z=w,source_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec))+'\n')
        for name,topic in (('LINK','/husky_link/status'),('GATE','/husky_nav/control_status'),
                           ('FILTER','/husky_control/safety/status'),('AI','/husky_nav/vision_control_status')):
            subscriptions.append(node.create_subscription(String,topic,lambda m,n=name:status(n,m),10))
        subscriptions.append(node.create_subscription(TwistStamped,'/husky_nav/filtered_cmd_vel_stamped',command,10))
        print('READ ONLY. Keep this running; reset/enable in another terminal, then press g ONCE on the robot.',flush=True)
        print('Stops 1 second after observed disarm, or after %ss. Log: %s.jsonl'%(args.duration,stem),flush=True)
        try:
            while time.monotonic()-started<args.duration:
                rclpy.spin_once(node,timeout_sec=.05)
                if disarmed_at is not None and time.monotonic()-disarmed_at>=1.:break
        except KeyboardInterrupt:
            pass
        finally:
            report=dict(read_only=True,physical_robot_validated=False,saw_gateway_arm=seen_arm,
                observed_sec=round(time.monotonic()-started,2),messages=dict(counts),
                max_abs_filtered_linear_mps=max_command,gate_reasons_while_armed=dict(gate_reasons),
                first_gate_closure_after_pass=first_closure,latest=latest,raw_log=str(stem.with_suffix('.jsonl')))
            path=stem.with_suffix('.json');path.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True);print('REPORT: '+str(path),flush=True)
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':
    main()
