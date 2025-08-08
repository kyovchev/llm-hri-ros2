from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import uvicorn


app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

robots = {
    "p": {"x": 2, "y": 3},
    "q": {"x": 5, "y": 1}
}

stations = {
    "1": {"x": 1, "y": 1},
    "2": {"x": 6, "y": 2},
    "3": {"x": 3, "y": 5},
}


@app.get("/", response_class=HTMLResponse)
async def get_ui(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", reload=True)
