"""First outer-route failure evidence must not alter control or write in Follow."""
import sys, tempfile, unittest, json, zlib
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ring_diagnostics as rd
import track_elements as te

class OuterFailureTests(unittest.TestCase):
    def setUp(self):
        self.c=rd.OuterFailureCapture()
        self.r=dict(session_token=3,ring_confirmed=True,outer_tracking=True,
                    outer_stage='READY',outer_rejection='NONE')
        self.s=dict(valid=True,source='ring_right')
        self.p=bytearray([7])*38400
    def frame(self,seq,valid=True):
        return self.c.observe(self.p,seq*10,seq,3,valid,self.s,self.r)
    def test_first_failure_and_next_frame_are_immutable_and_synchronized(self):
        with patch('builtins.open',side_effect=AssertionError('active write')):
            self.frame(9);self.p[0]=8;self.frame(10)
            self.r.update(outer_tracking=False,outer_rejection='MIN_ROWS')
            self.p[0]=11;self.frame(11,False)
            self.p[0]=12;self.frame(12,False)
            self.p[0]=13;self.assertFalse(self.frame(13,False))
        self.assertEqual([x[1] for x in self.c.meta],[10,11,12])
        self.assertEqual([x[0] for x in self.c.frames],[8,11,12])
        self.assertEqual(self.c.meta[1][5+self.c.FIELDS.index('outer_rejection')],'MIN_ROWS')
    def test_candidate_success_is_not_a_transmitted_outer_route(self):
        self.s['source']='road';self.frame(1);self.frame(2,False)
        self.assertFalse(self.c.failed)
    def test_session_identity_never_mixes(self):
        self.frame(1);self.r['session_token']=4
        self.assertFalse(self.frame(2,False));self.assertFalse(self.c.failed)
        self.c.reset();self.frame(3,False);self.assertFalse(self.c.failed)
    def test_exit_disarms_previous_outer_before_image(self):
        self.frame(1);self.r['ring_confirmed']=False;self.frame(2)
        self.r['ring_confirmed']=True;self.frame(3,False)
        self.assertFalse(self.c.failed)
    def test_save_only_frozen_manifest_and_crc_and_no_overwrite(self):
        self.frame(1);self.frame(2,False);self.frame(3,False)
        with tempfile.TemporaryDirectory() as d:
            base=str(Path(d)/'ring.bin')
            with self.assertRaises(RuntimeError):self.c.save(base,'build',False)
            self.assertEqual(list(Path(d).iterdir()),[])
            self.assertTrue(self.c.save(base,'build',True))
            m=json.loads(Path(base+'.outer.json').read_text())
            self.assertEqual([f['sequence'] for f in m['frames']],[1,2,3])
            for f in m['frames']:
                raw=Path(f['path']).read_bytes().split(b'\n',3)[3]
                self.assertEqual(len(raw),38400)
                self.assertEqual(zlib.crc32(raw)&0xffffffff,f['raw_crc32'])
            self.assertFalse(self.c.save(base,'build',True))
    def test_failed_write_retains_ram_and_never_commits_manifest(self):
        self.frame(1);self.frame(2,False)
        with tempfile.TemporaryDirectory() as d:
            base=str(Path(d)/'ring.bin')
            with patch.object(self.c,'_write_new',side_effect=OSError('full')):
                with self.assertRaises(OSError):self.c.save(base,'build',True)
            self.assertTrue(self.c.failed);self.assertFalse(self.c.persisted)
            self.assertFalse(Path(base+'.outer.json').exists())
            self.assertTrue(self.c.save(base,'build',True))
    def test_partial_write_retries_without_overwriting_completed_frame(self):
        self.frame(1);self.frame(2,False);self.frame(3,False)
        with tempfile.TemporaryDirectory() as d:
            base=str(Path(d)/'ring.bin');write=self.c._write_new;calls=[]
            def fail_second(path,*args):
                calls.append(path)
                if len(calls)==2:raise OSError('full')
                return write(path,*args)
            with patch.object(self.c,'_write_new',side_effect=fail_second):
                with self.assertRaises(OSError):self.c.save(base,'build',True)
            self.assertEqual(self.c.saved_slots,1)
            first=Path(base+'.outer-before.pgm').read_bytes()
            self.assertTrue(self.c.save(base,'build',True))
            self.assertEqual(Path(base+'.outer-before.pgm').read_bytes(),first)

    def test_session_integration_does_not_write_until_real_stop(self):
        from test_fast_main_ring_slope import main
        with tempfile.TemporaryDirectory() as d:
            session=main._RingDiagnosticsSession(rd.RingDiagnostics(),
                str(Path(d)/'ring.bin'),'build',raw_capture=self.c)
            with patch.object(main,'_preview_marker_image_buffer',return_value=(self.p,320)):
                for n,valid in ((1,True),(2,False),(3,False)):
                    session.observe(n*10,n,3,True,valid,self.s,self.r,None,raw_image=object())
                self.assertEqual(list(Path(d).iterdir()),[])
                session.observe(40,4,3,False,False,self.s,self.r,'PROFILE_INACTIVE')
            self.assertFalse(session.blocked,session.last_error)
            self.assertTrue(self.c.persisted)
            self.assertEqual(len(list(Path(d).glob('*.pgm'))),3)
            self.assertTrue(Path(session.last_saved_path+'.outer.json').exists())

    def test_new_active_reset_cannot_flush_raw_images(self):
        from test_fast_main_ring_slope import main
        with tempfile.TemporaryDirectory() as d:
            session=main._RingDiagnosticsSession(rd.RingDiagnostics(),
                str(Path(d)/'ring.bin'),'build',raw_capture=self.c)
            with patch.object(main,'_preview_marker_image_buffer',return_value=(self.p,320)):
                session.observe(10,1,3,True,True,self.s,self.r,None,raw_image=object())
                session.observe(20,2,3,True,False,self.s,self.r,None,raw_image=object())
                session.observe(30,3,3,True,False,self.s,self.r,'REVERSE_RESET',raw_image=object())
            self.assertTrue(session.blocked)
            self.assertTrue(self.c.failed)
            self.assertEqual(list(Path(d).glob('*.pgm')),[])

    def test_bad_image_cannot_replace_last_good_evidence(self):
        self.frame(1)
        self.assertFalse(self.c.observe(bytearray(1),20,2,3,False,self.s,self.r))
        self.assertFalse(self.c.failed);self.assertEqual(self.c.meta[0][1],1)
    def tracker(self):
        t=te.TrackElementsTracker({},require_calibration=True)
        t._state=te.STATE_INSIDE;t._inner_route_established=True
        t._current_valid=True;t._session_active=True;t._calibration_ready=True
        t._cal_check_ready=True;t._ring_width_calibrated=True
        t._ring_cache_frozen=True;t._side=te.SIDE_RIGHT
        t._result['ring_target_rejection']='NEAR_SUPPORT'
        t._selected_route_ms=0;t._last_now_ms=10
        return t
    def test_real_outer_build_rejection_survives_inner_restore(self):
        t=self.tracker()
        def fail(*a):t._result['ring_target_rejection']='REFERENCE_GAP';return False
        t._build_locked_target_rows=fail
        self.assertFalse(t._build_outer_route({},{}))
        self.assertEqual(t._result['ring_target_rejection'],'NEAR_SUPPORT')
        self.assertEqual(t._result['outer_rejection'],'REFERENCE_GAP')
        self.assertEqual(t._result['outer_stage'],'BUILD')
    def test_handoff_rejection_survives_output_clear(self):
        t=self.tracker()
        def ok(*a):t._result['target_center2']=100;return True
        t._build_locked_target_rows=ok;t._filter_outer_route_rows=lambda *a:True
        t._handoff_continuous=lambda:False
        self.assertFalse(t._build_outer_route({},{}))
        self.assertEqual(t._result['outer_rejection'],'HANDOFF_CONTINUITY')
        self.assertEqual(t._result['target_count'],0)
    def test_success_does_not_relabel_legacy_reject_counters(self):
        t=self.tracker()
        t._build_locked_target_rows=lambda *a:True
        t._filter_outer_route_rows=lambda *a:True;t._handoff_continuous=lambda:True
        self.assertTrue(t._build_outer_route({},{}))
        self.assertTrue(t._result['outer_tracking'])
        self.assertEqual(t._result['outer_stage'],'READY')
        self.assertEqual(t._result['outer_rejection'],'NONE')

if __name__=='__main__':unittest.main()
