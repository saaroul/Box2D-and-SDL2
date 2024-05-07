// Box2D-and-SDL2.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "box2d/box2d.h"
#include <SDL.h>

const int WIDTH = 1280;
const int HEIGHT = 720;
const int MET2PIX = 10; // box2d meter to pixel scale 80px = 1m

const int SCALED_WIDTH = WIDTH / MET2PIX;
const int SCALED_HEIGHT = HEIGHT / MET2PIX;

const float RAD2DEG = 180.0f / b2_pi; // 1 rad = 180 / pi = 57.296 deg

void DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius);


int main(int argc, char* argv[])
{
	// Ground size and pos
	float ground_x_size = 20.0f, ground_y_size = 0.2f;
	float ground_x = 0.0f, ground_y = -10.0f;
	float ground_angle = -0.01f * b2_pi;


	// Box size and pos
	float box_x_size = 1.0f, box_y_size = 1.0f;
	float box_x = 0.0f, box_y = 10.0f;
	float box_angle = 0.0f;
	
	// Time step for box2d simulation
	float timeStep = 1.0f / 60.0f; // 60 times per second (16ms per step)

	// Iterations for box2d simulations. Higher means better accuracy but more overhead
	int32 velocityIterations = 6;
	int32 positionIterations = 2;

	// box2d world definition
	b2Vec2 gravity(0.0f, -9.81f);
	b2World world(gravity);
	
	// World Ground
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(ground_x, ground_y);
	groundBodyDef.angle = ground_angle;
	//groundBodyDef.fixedRotation = true;
	b2Body* groundBody = world.CreateBody(&groundBodyDef);

	b2PolygonShape groundBox;
	groundBox.SetAsBox(ground_x_size, ground_y_size);

	groundBody->CreateFixture(&groundBox, 0.0f);	

	// Falling box def
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(box_x, box_y);
	b2Body* body = world.CreateBody(&bodyDef);

	b2PolygonShape dynamicBox;
	dynamicBox.SetAsBox(box_x_size, box_y_size);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 0.01f;

	body->CreateFixture(&fixtureDef);


	// SDL2 Initializations
	SDL_Window* window = NULL;
	SDL_Surface* screenSurface = NULL;
	SDL_Renderer* renderer = NULL;
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
		return 1;
	}
	window = SDL_CreateWindow(
		"hello_sdl2",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		WIDTH, HEIGHT,
		SDL_WINDOW_SHOWN
	);
	if (window == NULL) {
		fprintf(stderr, "could not create window: %s\n", SDL_GetError());
		return 1;
	}

	renderer = SDL_CreateRenderer(
		window,	-1,
		SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer == NULL)
	{
		fprintf(stderr, "could not create renderer: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Surface *temp = SDL_CreateRGBSurface(0, box_x_size * MET2PIX * 2, box_y_size * MET2PIX * 2, 32, 0, 0, 0, 0);
	SDL_FillRect(temp, NULL, SDL_MapRGB(temp->format, 0x1F, 0x6F, 0x0F));
	SDL_Texture* box_texture = SDL_CreateTextureFromSurface(renderer, temp);
	int boxF_width = temp->w;
	int boxF_height = temp->h;
	SDL_FreeSurface(temp);

	SDL_Surface *ground_temp = SDL_CreateRGBSurface(0, ground_x_size * MET2PIX * 2, ground_y_size * MET2PIX * 2, 32, 0, 0, 0, 0);
	SDL_FillRect(ground_temp, NULL, SDL_MapRGB(ground_temp->format, 0xFF, 0xFF, 0xFF));
	SDL_Texture* ground_texture = SDL_CreateTextureFromSurface(renderer, ground_temp);
	int groundF_width = ground_temp->w;
	int groundF_height = ground_temp->h;
	SDL_FreeSurface(ground_temp);

	bool keep_window_open = true;
	while (keep_window_open)
	{
		SDL_Event e;
		while (SDL_PollEvent(&e) > 0)
		{
			switch (e.type)
			{
			case SDL_QUIT:
				keep_window_open = false;
				break;
			}
		}
		world.Step(timeStep, velocityIterations, positionIterations);
		b2Vec2 position = body->GetPosition();
		float angle = body->GetAngle();
		b2Vec2 b2ground_position = groundBody->GetPosition();
		float b2ground_angle = groundBody->GetAngle();
		printf("BOX: %4.2f %4.2f %4.2f\n", position.x, position.y, angle);


		SDL_RenderClear(renderer);

		SDL_Rect box_dst;
		box_dst.x = (position.x - box_x_size) * MET2PIX + WIDTH/2;
		box_dst.y = - (position.y + box_y_size) * MET2PIX + HEIGHT / 2;
		box_dst.w = boxF_width;
		box_dst.h = boxF_height;

		SDL_Rect ground_dst;
		ground_dst.x = (b2ground_position.x - ground_x_size) * MET2PIX + WIDTH / 2;
		ground_dst.y = - (b2ground_position.y + ground_y_size) * MET2PIX + HEIGHT / 2;
		ground_dst.w = groundF_width;
		ground_dst.h = groundF_height;

		SDL_RenderCopyEx(renderer, box_texture, NULL, &box_dst, -angle * RAD2DEG, NULL, SDL_FLIP_NONE);
		SDL_RenderCopyEx(renderer, ground_texture, NULL, &ground_dst, -b2ground_angle * RAD2DEG, NULL, SDL_FLIP_NONE);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);

		//SDL_RenderCopyEx(renderer, box_texture, NULL, )
		SDL_RenderPresent(renderer);
		SDL_Delay(16);
	}
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}