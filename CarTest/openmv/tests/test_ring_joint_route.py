"""Production calibrated-width admission and mirrored final V3 steering."""
import sys
from pathlib import Path
from array import array
import types
import unittest

HERE = Path(__file__).resolve().parent
for path in (HERE, HERE.parent):
    if str(path) not in sys.path: sys.path.insert(0, str(path))
import test_fast_main_ring_slope as runtime
import main
import config
import vision_protocol


def route_vector(side, left, right, width_flag=None):
    main._load_fast_runtime()
    main._load_singleedge_runtime()
    main._load_track_elements_runtime()
    te = main.track_elements
    if width_flag is None: width_flag = te.TARGET_WIDTH_FROM_CALIBRATION
    flags = width_flag | (te.TARGET_OBSERVED_LEFT | te.TARGET_INFERRED_RIGHT
                          if side == te.SIDE_LEFT else
                          te.TARGET_OBSERVED_RIGHT | te.TARGET_INFERRED_LEFT)
    rows = array("i")
    edges = runtime._SingleEdgeTracker(config)
    edges._frame_scan_done = True
    for y in range(config.SURFACE_ROI_FLOOR_Y, 60):
        rows.extend((y, left, right, flags))
        slot = side * 60 + y
        edges._row_status[slot] = 1
        edges._row_edge2[slot] = left * 2 - 1 if side == te.SIDE_LEFT else right * 2 + 1
    tracker = types.SimpleNamespace(min_target_rows=5, max_cache_gap=3,
        require_calibration=True, ring_calibration_ready=True,
        ring_calibration_check_ready=True)
    element = dict(control_eligible=True, target_valid=True,
        target_rows=rows, target_count=len(rows)//4, target_stride=4,
        target_side=side, side=side, observed_rows=52, inferred_rows=52)
    route = {}
    output = array("i", [0] * (60 * main.fast_single_edge.ROW_STRIDE))
    return tracker, element, edges, route, output


def converted_error(side, left, right):
    tracker, element, edges, route, output = route_vector(side, left, right)
    count = main._fill_ring_control_route(element, tracker, edges, False, True,
                                          route, output, config)
    if not count: raise AssertionError("calibrated ring route rejected")
    selection = dict(valid=True, source=("ring_left" if side==0 else "ring_right"),
                     result=route, lateral_error=route["lateral_error"],
                     boundary_source=route["boundary_source"], inferred=True)
    info = main._FastRowWeightWorkspace().aggregate(selection, route, config)
    if not info.get("valid"): raise AssertionError(info)
    return int(info["position_error"])


class CalibratedRingRouteTests(unittest.TestCase):
    def test_calibrated_both_sides_reach_real_weighted_error(self):
        left_error=converted_error(0,10,34)
        right_error=converted_error(1,45,69)
        self.assertLess(left_error,0)
        self.assertGreater(right_error,0)
        self.assertLessEqual(abs(left_error+right_error),1)

    def test_real_calibrated_tracker_target_rows_are_accepted_by_converter(self):
        from test_track_elements import TrackElementsHarness
        main._load_track_elements_runtime();te=main.track_elements
        class Calibration:
            ready=True
            reason="CALIBRATION_READY"
            width_valid=bytearray([0]*8+[1]*52)
            width_cells=array("i",[0]*8+[31]*52)
        for side,left,right in ((te.SIDE_LEFT,10,40),(te.SIDE_RIGHT,39,69)):
            with self.subTest(side=side):
                h=TrackElementsHarness()
                h.tracker=te.TrackElementsTracker({},Calibration(),True)
                rows=array("i")
                for y in range(8,60):rows.extend((y,25,55,7))
                for _ in range(3):h.tracker.observe_ring_calibration(rows,52,4)
                h.prime();h.frame(left=left,right=right)
                self.assertTrue(h.tracker._freeze_ring_width())
                h.tracker._side=side
                h.tracker._state=te.STATE_INSIDE
                self.assertTrue(h.tracker._build_target_rows(
                    h.mask(lambda y:left,lambda y:right),
                    {"reason":"OK","complete":True},side))
                h.tracker._publish();result=h.tracker.result
                self.assertEqual(result["target_rows"][3] & (
                    te.TARGET_WIDTH_FROM_CALIBRATION|te.TARGET_WIDTH_FROM_CACHE),
                    te.TARGET_WIDTH_FROM_CALIBRATION)
                route={};converted=array("i",[0]*300)
                edge_owner=types.SimpleNamespace(_row_status=h.tracker._current_status,
                    _row_edge2=h.tracker._current_edge2)
                count=main._fill_ring_control_route(result,h.tracker,edge_owner,True,
                    False,route,converted,config)
                self.assertGreaterEqual(count,5)
                self.assertLess(route["lateral_error"],0) if side==0 else self.assertGreater(route["lateral_error"],0)

    def test_main_restores_unmodified_ordinary_route_only_after_feedback(self):
        import contextlib,io
        from unittest.mock import patch
        main._load_fast_runtime();main._load_singleedge_runtime();main._load_track_elements_runtime()
        te=main.track_elements;instances=[]
        class Feedback(runtime._ConfirmedRingNoTarget):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs);self.calls=[];self.f=0;instances.append(self)
            def update(self,*args,**kwargs):
                self.f+=1
                return self.result
            def observe_route_outcome(self,now,ring_selected,ordinary_valid):
                self.calls.append((ring_selected,ordinary_valid))
                if self.f==3 and ordinary_valid:
                    self.result.update(state="REACQUIRE",state_id=te.STATE_REACQUIRE,
                        ring_confirmed=False,candidate=False,control_eligible=False,
                        reason="ORDINARY_ROUTE_REACQUIRED")
                    return True
                return False
        clock=runtime._Time()
        controls=bytes(main.reverse_control.pack_frame(0,100))+bytes(main.reverse_control.pack_frame(1,101))+bytes(main.reverse_control.pack_profile_state(11,3,main.reverse_control.PROFILE_SESSION_ACTIVE,session_token=1))
        uart=runtime._UART(clock,stop_after=3,rx_data=controls)
        with contextlib.ExitStack() as stack:
            for target,name,value in (
                (main,"_configure_gray_sensor",lambda *_a,**_k:1),
                (main,"sensor",runtime._Sensor()),(main,"time",clock),
                (main.fast_gray,"FastGrayMasks",runtime._GrayBuilder),
                (main.fast_path,"FastPathDetector",runtime._PathDetector),
                (main.fast_boundary,"FastBoundaryEvidence",runtime._BoundaryValidator),
                (main.fast_single_edge,"FastSingleEdgeTracker",runtime._SingleEdgeTracker),
                (main.track_elements,"TrackElementsTracker",Feedback),
                (main.pyb,"UART",lambda *_a,**_k:uart),
                (main,"_validate_selected_fast_boundary",lambda selection,_r,_v:(selection,{"x":50.0,"y":40.0,"kind":"ROAD"},{"valid":True,"reason":"TEST_VALID"})),
                (config,"DEBUG_ENABLED",False),(config,"OVERLAY_ENABLED",False)):
                stack.enter_context(patch.object(target,name,value,create=True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            main._main_fast(duration_s=1)
        self.assertEqual(len(uart.frames),3)
        self.assertEqual(instances[0].calls,[(False,True)]*3)
        valid=[bool(frame[6]&vision_protocol.FLAG_VALID) for frame in uart.frames]
        self.assertEqual(valid,[False,False,True])
        self.assertNotEqual(int.from_bytes(uart.frames[-1][8:10],"little",signed=True),0)
        self.assertTrue(all(frame[11]==3 for frame in uart.frames))

    def test_mixed_tracker_or_diagnostic_bundle_is_rejected(self):
        from unittest.mock import patch
        main._load_track_elements_runtime();main._load_ring_runtime()
        for module,attribute in ((main.track_elements,"JOINT_ROUTE_API_VERSION"),
                                 (main.ring_diagnostics,"FORMAT_VERSION")):
            with patch.object(module,attribute,0):
                with self.assertRaises(RuntimeError):main._validate_fast_config()

    def test_calibration_flags_do_not_bypass_map_or_parked_gate(self):
        for map_ready, parked in ((False,True),(True,False),(False,False)):
            tracker, element, edges, route, output=route_vector(0,10,34)
            tracker.ring_calibration_ready=map_ready
            tracker.ring_calibration_check_ready=parked
            self.assertEqual(main._fill_ring_control_route(element,tracker,edges,
                False,True,route,output,config),0)
        te=main.track_elements
        for flags in (0,te.TARGET_WIDTH_FROM_CACHE,
                      te.TARGET_WIDTH_FROM_CACHE|te.TARGET_WIDTH_FROM_CALIBRATION):
            tracker, element, edges, route, output=route_vector(0,10,34,flags)
            self.assertEqual(main._fill_ring_control_route(element,tracker,edges,
                False,True,route,output,config),0)

    def test_missing_or_ambiguous_current_edge_cannot_be_admitted(self):
        for state in (0,2):
            tracker, element, edges, route, output=route_vector(1,45,69)
            edges._row_status=bytearray([state]*120)
            self.assertEqual(main._fill_ring_control_route(element,tracker,edges,
                False,True,route,output,config),0)

    def test_mirrored_calibrated_routes_reach_actual_main_v3_uart(self):
        te=main.track_elements if main.track_elements else None
        main._load_track_elements_runtime();te=main.track_elements
        runner=runtime.FastMainRingSlopeWireTests(methodName=
            "test_fresh_profile_b_ring_target_reaches_the_existing_uart_chain")
        frames=[]
        packets=[]
        for side,left,right in ((te.SIDE_LEFT,10,34),(te.SIDE_RIGHT,45,69)):
            class Mirrored(runtime._ActiveRingElements):
                def __init__(self,*args,**kwargs):
                    super().__init__(*args,**kwargs)
                    self.require_calibration=True
                    self.rows=array("i")
                    flags=te.TARGET_WIDTH_FROM_CALIBRATION | (
                        te.TARGET_OBSERVED_LEFT|te.TARGET_INFERRED_RIGHT if side==te.SIDE_LEFT
                        else te.TARGET_OBSERVED_RIGHT|te.TARGET_INFERRED_LEFT)
                    for y in range(config.SURFACE_ROI_FLOOR_Y,60):
                        self.rows.extend((y,left,right,flags))
                    self.result.update(side=side,target_side=side,target_rows=self.rows,
                                       target_center2=left+right)
            frame,log,commits=runner._run_profile_element_case(Mirrored,3)
            self.assertEqual(frame[11],3)
            self.assertEqual(frame[6]&vision_protocol.FLAG_VALID,vision_protocol.FLAG_VALID)
            self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:14]),
                             int.from_bytes(frame[14:16],"little"))
            error=int.from_bytes(frame[8:10],"little",signed=True)
            self.assertLess(error,0) if side==0 else self.assertGreater(error,0)
            self.assertIn("source=ring_left" if side==0 else "source=ring_right",log)
            self.assertEqual(commits,[])
            frames.append(error)
            packets.append(frame)
        self.assertLessEqual(abs(sum(frames)),1)
        self.wire_frames=packets

if __name__=="__main__":unittest.main()
