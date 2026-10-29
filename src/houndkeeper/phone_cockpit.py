"""The phone cockpit ([Swt]): DimOS's Cockpit laid out for a phone, with houndkeeper's own pages as extensions
(cockpit/extensions/, loaded by the relay's --extensions-dir, [Eir]): the camera over touch driving ([Xzt]), and the
Procedures ([vRJ]), Status ([F2q]), map, Mind ([RNC]), Morning ([Ndt]) and Area ([daF]) pages. Compose onto the robot:

    dimos run unitree-go2 houndkeeper.battery houndkeeper.phone-cockpit --local-relay --extensions-dir cockpit/extensions \
        --viewer none

--viewer none: DimOS's default Rerun viewer serves the camera, lidar and map on 0.0.0.0:9876 without a login, to
every machine on the house network and the tailnet; the phone has the camera through the relay already.

The Mind page needs the tidy module in the same run, publishing (HK_MIND=1): add houndkeeper.tidy, as
sim/dimos/tidy_run.sh runs it, or the robot's tidy blueprint. A recorded night (--record) keeps the summary ([Fho]) and
plays it again through the same page ([B4u]); --speed (the replay's) watches it faster:

The Morning page is the latest night's morning log ([uoW]: morning_log.py writes it into the night's directory;
cockpit_data.MorningLogFeed reads the newest under HK_MORNING_RUNS, default data/runs/dimos_tidy).

    dimos --replay-db recordings/<run-id>/memory.db --viewer none run replay houndkeeper.phone-cockpit --local-relay \
        --extensions-dir cockpit/extensions --speed 4       # from this repository
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.nav_msgs.Path import Path
from dimos.web.cockpit import Channel, ChannelRequest, Col, Map2D, Panel, Teleop, Video, _request_of, cockpit

from houndkeeper import mind
from houndkeeper.cockpit_data import CockpitData, LinkStatus, MindFeed, MorningLogFeed, ProcedureState
from houndkeeper.operating_area_module import AreaFeed


@dataclass(frozen=True)
class Procedures(Panel):
    """the calibration procedures, run live (cockpit/extensions/hk-procedures.js)"""
    kind: ClassVar[str] = "hk-procedures"
    title: str = field(default="Procedures", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("procedure_state", ProcedureState, max_hz=2.0, delivery="latest", resend_on_subscribe=True),
                Channel("procedure_cmd", dict, dir="tx", publish="shared", max_hz=5.0))

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


@dataclass(frozen=True)
class RobotStatus(Panel):
    """battery, power, the camera link, the robot network's isolation (cockpit/extensions/hk-status.js)"""
    kind: ClassVar[str] = "hk-status"
    title: str = field(default="Status", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("link_status", LinkStatus, max_hz=2.0, delivery="latest", resend_on_subscribe=True),)

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


@dataclass(frozen=True)
class MindView(Panel):
    """what he thinks, live ([RNC]): the floor map with the routes, the candidates and the goal with its reason ([Ttd]),
    and the candidates outlined over the camera ([D1U]) (cockpit/extensions/hk-mind.js). Reads only: every channel is
    rx ([ls3]). Channel order: mind, costmap, then path and video when bound (named in params)."""
    kind: ClassVar[str] = "hk-mind"
    costmap: str = "global_costmap"
    path: str | None = "path"                  # the planner's route to the goal (the Go2 navigation's path stream)
    video: str | None = "color_image"
    title: str = field(default="Mind", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        out = [Channel("mind", dict, max_hz=1 / mind.PERIOD_S, delivery="latest", resend_on_subscribe=True)]
        if self.path is not None:              # as Map2D declares it, so the two merge
            out.append(Channel(self.path, Path, encoding="path.json.v1", delivery="latest", max_hz=10.0, paced=True,
                               resend_on_subscribe=True))
        return tuple(out)

    def _panel_params(self) -> dict:
        return {k: v for k, v in (("costmap", self.costmap), ("path", self.path), ("video", self.video)) if v is not None}

    def _channel_requests(self):
        c = self._channels(); out = [_request_of(c[0]), ChannelRequest(self.costmap, "rx", "costmap.zlib.v1", 2.0, delivery="latest")]
        out += [_request_of(x) for x in c[1:]]
        if self.video is not None:
            out.append(ChannelRequest(self.video, "rx", "jpeg.v1", 15.0, {"quality": 75}, delivery="latest"))
        return tuple(out)


@dataclass(frozen=True)
class MorningLog(Panel):
    """the butler's morning log, the latest night's ([Ndt]; cockpit/extensions/hk-morning-log.js): his prose, a crop,
    time and place for each find, the night's figures with their sources. Reads only: its one channel is rx."""
    kind: ClassVar[str] = "hk-morning-log"
    title: str = field(default="Morning", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("morning_log", dict, max_hz=0.2, delivery="latest", resend_on_subscribe=True),)

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


@dataclass(frozen=True)
class AreaPage(Panel):
    """the operating area drawn and edited on the house map ([JVr] [daF]), with the warning that it is no safety barrier
    ([HZO]) (cockpit/extensions/hk-area.js). Sends only a confirmed save, which the robot side validates, stores with the
    map and logs (operating_area_module.AreaFeed)."""
    kind: ClassVar[str] = "hk-area"
    title: str = field(default="Area", kw_only=True)

    def _channels(self) -> tuple[Channel, ...]:
        return (Channel("area_state", dict, max_hz=1.0, delivery="latest", resend_on_subscribe=True),
                Channel("area_cmd", dict, dir="tx", publish="shared", max_hz=2.0))

    def _channel_requests(self):
        return tuple(_request_of(c) for c in self._channels())


def phone_layout():
    # 200 ms deadman: 3 missed sends at 15 Hz; the default 300 plus the robot's own halt overran 0.6 s ([Mj2], 3 Oct 2026)
    return dict(layout=Col(Video("color_image"), Teleop(title="Drive", watchdog_ms=200.0), shares=[3, 2]),
                pages=[Procedures(), RobotStatus(), Map2D(costmap="global_costmap", pose="odom", title="Map"), MindView(),
                       MorningLog(), AreaPage()])


phone_cockpit = autoconnect(cockpit(**phone_layout()), CockpitData.blueprint(), MindFeed.blueprint(), MorningLogFeed.blueprint(),
                            AreaFeed.blueprint())

# the Area page alone, with no robot or sim: draw the operating area on the house map from the laptop
#   dimos --robot-id area --relay-url <relay> --viewer none run houndkeeper.area      (HK_MAP_DIR: the map directory)
area = autoconnect(cockpit(layout=AreaPage()), AreaFeed.blueprint())
