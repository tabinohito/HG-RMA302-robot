"""Main-thread power interlocks, input servicing and ESC driving."""
import signal
import time
from .control import DriveController, Output
from .esc import EscPair, EscFault
from .gamepad import Gamepad
from .power import PowerLink, PowerError
from .power_settings import supply_state

INPUT_FAILURES={'DISCONNECTED','INVALID_INPUT','INPUT_TIMEOUT','GAMEPAD_POLL_TIMEOUT','LOOP_TIMEOUT'}


def power_gate(config, power, pair, now):
    if not power.ready:
        return 'POWER_CONNECTING'
    if power.estopped:
        return 'STOP_LATCHED'
    d=power.status
    if d['ina']!='1' or int(d['sample_age_ms'])>300:
        return 'POWER_SENSOR_FAULT'
    # INA bus may sit on either side of the SSR. Wheel buses decide whether
    # drive voltage is present; a relay command is not contact feedback.
    if pair is None:
        return 'ESC_CONNECTING'
    if int(d['mv']) > max(w.bus_limits_mv[1] for w in pair.wheels):
        raise PowerError('INA226 bus overvoltage')
    states=[]
    for w in pair.wheels:
        if w.last_status is None or now-w.last_status_at>.5:
            return 'WAIT_ESC_STATUS'
        mv=int(w.last_status['bus_mV'])
        states.append(supply_state(mv,*w.bus_limits_mv,config.power))
    if 'OVERVOLTAGE' in states:
        raise PowerError('ESC bus overvoltage')
    if 'USB_ONLY' in states:
        return 'USB_ONLY'
    if d['relay']!='1' or any(s!='READY' for s in states):
        return 'WAIT_POWER'
    return None


def led_state(reason):
    if reason in ('STOP_LATCHED', *INPUT_FAILURES): return 'STOPPED'
    if reason=='USB_ONLY': return 'USB_ONLY'
    if reason in ('WAIT_POWER','WAIT_ESC_STATUS'): return 'WAIT_POWER'
    if reason in ('POWER_SENSOR_FAULT','ERROR'): return 'FAULT'
    if reason in ('POWER_CONNECTING','ESC_CONNECTING'): return 'BOOT'
    if reason and (reason.startswith('RECOVER') or reason=='FAULT_STATUS_WAIT'): return 'RECOVERING'
    if reason in ('DRIVING','WAITING_FOR_WHEEL','WAITING_FOR_BOTH_WHEELS'): return 'DRIVING'
    return 'READY'


class _Finished(Exception):
    pass


def run_powered(config, *, seconds=None, logger=None):
    from .cli import report_esc_events
    pair=power=pad=None
    controller=None
    output=Output()
    display_reason='POWER_CONNECTING'
    stop_reason='finished'
    stopped=False
    previous_reset=False
    last_print=last_log=-float('inf')
    previous_reason=None
    started=time.monotonic()

    def interrupted(signum, frame):
        nonlocal stopped,stop_reason
        stopped=True;stop_reason=signal.Signals(signum).name
    handlers={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}

    def drop_pair():
        nonlocal pair
        if pair is not None:
            old,pair=pair,None
            errors=old.close()
            report_esc_events(old,logger)
            if logger: logger.record('esc_cleanup',stop_errors=errors)
            if errors: print('ESC停止送信:', '; '.join(errors),flush=True)

    def emergency():
        # Attempt both paths even if one cable or write has failed.
        try:
            if power: power.estop()
        finally:
            drop_pair()

    def handle_input(sample,now):
        nonlocal output,previous_reset,display_reason
        output=controller.update(sample,now)
        neutral=(abs(sample.left_stick)<=config.drive.deadzone and abs(sample.right_stick)<=config.drive.deadzone
                 and not sample.dpad_x and not sample.dpad_y)
        reset_edge=sample.reset and not previous_reset
        previous_reset=sample.reset
        if sample.estop or output.reason in INPUT_FAILURES:
            emergency()
        elif reset_edge and neutral and power and power.estopped:
            power.clear()
        # Process brief stop/neutral frames before the next frame, preserving
        # B presses even if press and release arrive in the same USB read.
        if pair and (not output.enabled or not(output.left_rpm or output.right_rpm)):
            pair.update(output,now)

    def service():
        """Also called during stopped ESC initialization; never arms motors."""
        nonlocal stop_reason,output,display_reason
        now=time.monotonic()
        if stopped or (seconds is not None and now-started>=seconds):
            if not stopped: stop_reason='duration'
            raise _Finished
        frames=pad.read()
        now=time.monotonic()
        for sample in frames:
            handle_input(sample,now)
            if logger: logger.snapshot(sample,output,pair,now,kind='input_frame',power=power)
        handle_input(pad.state,now)
        power.poll(now,led_state(display_reason))
        if power.estopped:
            drop_pair()
            output=controller.inhibit('STOP_LATCHED')
            display_reason=output.reason
        for message in power.notices:
            print(message,flush=True)
            if logger: logger.record('notice',message=message)
        power.notices.clear()

    try:
        pad=Gamepad(config.gamepad,event_timeout_s=config.drive.input_timeout_s)
        controller=DriveController(config.drive,require_input_updates=pad.requires_event_updates)
        device=pad.device
        print(f'Gamepad: {device.name} {device.path}',flush=True)
        power=PowerLink(config.power)
        print(f'Power: serial={config.power.serial or "(path)"} → {power.port.path}',flush=True)
        print('左右スティック縦=各輪、十字キー=前後・旋回。B=ESC停止＋SSR OFF・停止保持。',flush=True)
        print('中立でStart=非常停止解除（SSRはOFFのまま）。駆動電源は物理ボタンで投入。約5 VはUSB給電待機。',flush=True)
        print('LED: 待機=青、READY=水色、走行=緑、復帰待ち=橙、非常停止/異常=赤。Ctrl-C=SSR OFF・終了。',flush=True)
        if logger:
            logger.record('gamepad',name=device.name,path=device.path)
            logger.record('power_connected',serial=config.power.serial,path=power.port.path)
        while True:
            try:
                service()
                now=time.monotonic()
                if pair is None and power.ready and not power.estopped:
                    display_reason='ESC_CONNECTING'
                    output=controller.inhibit(display_reason)
                    # service reads B/input and renews the power lease every
                    # wait cycle; blocked serial setup cannot freeze e-stop.
                    pair=EscPair(config,service=service,allow_standby=True)
                    output=controller.inhibit('CENTER_CONTROLS')
                    now=time.monotonic()
                    if logger: logger.record('esc_connected',devices={side:{'serial':getattr(config,side+'_serial'),'path':p.path}
                                             for side,p in zip(('left','right'),pair.ports)})
                if pair and power.status and power.status['relay']=='0' and any(w.phase not in ('stopped','stopping') for w in pair.wheels):
                    drop_pair()
                    output=controller.inhibit('WAIT_POWER')
                if pair and all(w.phase=='stopped' for w in pair.wheels):
                    pair.poll_standby(now)
                reason=power_gate(config,power,pair,now)
                if reason:
                    output=controller.inhibit(reason)
                    if reason=='POWER_SENSOR_FAULT': emergency()
                    elif pair: pair.update(output,now)
                    display_reason=reason
                elif pair:
                    pair.update(output,now)
                    report_esc_events(pair,logger)
                    recovery=pair.recovery_display(now)
                    display_reason=(recovery[0] if recovery else pair.speed_sync.reason or output.reason)
            except EscFault as exc:
                # A physical OFF can reach the ESC before Pico's next 100 ms
                # status. Stop both and return to standby, never replay motion.
                side=next((w for w in pair.wheels if w.port.path==exc.path),None) if pair else None
                if (exc.status['fault']=='bus_voltage' and side is not None and
                        0<=int(exc.status['bus_mV'])<side.bus_limits_mv[0] and
                        exc.status['MOE']=='0' and int(exc.status['CCER'],0)==0):
                    drop_pair();output=controller.inhibit('WAIT_POWER');display_reason='WAIT_POWER'
                    continue
                raise
            now=time.monotonic()
            if logger and now-last_log>=.02:
                logger.snapshot(pad.state,output,pair,now,power=power);last_log=now
                warning=logger.warning()
                if warning: print(warning,flush=True)
            if now-last_print>=.25 or display_reason!=previous_reason:
                p=power.snapshot(now)
                voltage='--' if p['voltage_v'] is None else f"{p['voltage_v']:.2f} V"
                amps='範囲外' if p['current_saturated'] else '--' if p['current_a'] is None else f"{p['current_a']:+.3f} A"
                esc=''
                if pair:
                    esc=' ESC='+','.join(f"{(w.last_status or {}).get('bus_mV','?')}mV/{(w.last_status or {}).get('state','?')}"
                                          for w in pair.wheels)
                print(f'{display_reason:22} {voltage} {amps} SSR={int(p["ssr_command_on"])} '
                      f'RPM_cmd L={output.left_rpm:+.2f} R={output.right_rpm:+.2f}{esc}',flush=True)
                last_print=now;previous_reason=display_reason
            time.sleep(max(0,.005-(time.monotonic()-now)))
    except _Finished:
        pass
    except BaseException:
        stop_reason='error'
        try: emergency()
        except Exception as exc: print(f'非常停止送信エラー: {exc}',flush=True)
        if logger: logger.snapshot(pad.state if pad else None,output,pair,time.monotonic(),kind='error_snapshot',power=power)
        raise
    finally:
        try:
            drop_pair()
        finally:
            try:
                if power:
                    errors=power.close()
                    if logger: logger.record('power_cleanup',stop_errors=errors)
                    for error in errors: print(error,flush=True)
            finally:
                if pad: pad.close()
                for s,handler in handlers.items(): signal.signal(s,handler)
                if logger: logger.record('drive_exit',reason=stop_reason)
