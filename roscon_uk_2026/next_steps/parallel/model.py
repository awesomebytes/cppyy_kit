"""Validation is complete before native storage is constructed."""
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter


class Detection(BaseModel):
    model_config = ConfigDict(strict=True, frozen=True, extra="forbid", allow_inf_nan=False,
                              revalidate_instances="always")
    x: float = Field(ge=-1000, le=1000)
    y: float = Field(ge=-1000, le=1000)
    z: float = Field(ge=-1000, le=1000)
    confidence: float = Field(ge=0, le=1)
    frame: int = Field(ge=0, le=4095)


RECORDS = TypeAdapter(list[Detection])


def validate(records, frames=64):
    if type(frames) is not int or not 1 <= frames <= 4096:
        raise ValueError("frames must be an integer in [1, 4096]")
    models = RECORDS.validate_python(records)
    if any(d.frame >= frames for d in models):
        raise ValueError("record frame is outside this batch's frame count")
    return models
