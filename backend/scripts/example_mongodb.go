package main

import (
    "context"
    "fmt"
    "go.mongodb.org/mongo-driver/bson"
    "github.com/fatih/color"
    // "time"

    "go.mongodb.org/mongo-driver/mongo"
    "go.mongodb.org/mongo-driver/mongo/options"
    // "go.mongodb.org/mongo-driver/mongo/readpref"
)

type Trainer struct {
    Name string
    Age  int
    City string
}

type TrainerE struct {
    Name string
    Age  int
    City string
    Cmt  string
}

func play(){
    client, err := mongo.Connect(context.TODO(), options.Client().ApplyURI("mongodb://127.0.0.1:27017"))
    if err == nil {
        color.Green("Connected to MongoDB!")
    }
    defer client.Disconnect(context.TODO())
    err = client.Ping(context.TODO(), nil)
    if err == nil {
        color.Green("MongoDB pinged")
    }

    collection := client.Database("test").Collection("trainers")
    
    /* Create*/
    
    ash := Trainer{"Ash", 10, "Pallet Town"}
    insertResult, _ := collection.InsertOne(context.TODO(), ash)
    fmt.Println("Inserted a single document: ", insertResult.InsertedID)
    misty := Trainer{"Misty", 10, "Cerulean City"}
    brock := Trainer{"Brock", 15, "Pewter City"}

    trainers := []interface{}{misty, brock}
    insertManyResult, _ := collection.InsertMany(context.TODO(), trainers)
    fmt.Println("Inserted multiple documents: ", insertManyResult.InsertedIDs) 
    te := TrainerE{"Ash", 10, "Pallet Town", "example"}
    insertResult, _ = collection.InsertOne(context.TODO(), te)
    fmt.Println("Inserted a single document: ", insertResult.InsertedID)

    /* Update 
    
    filter := bson.D{{"name", "Ash"}}

    update := bson.D{
        {"$inc", bson.D{
            {"age", 100},
        }},
    }
    updateResult, _ := collection.UpdateMany(context.TODO(), filter, update)
    fmt.Println(updateResult)
    */
    
    /* Read single*/
    
    var result Trainer
    filter := bson.D{{"name", "Ash"}}
    _ = collection.FindOne(context.TODO(), filter).Decode(&result)
    fmt.Printf("Found a single document: %+v\n", result)
    
    /*Read multiple
    
    options := options.Find()
    options.SetLimit(2)
    filter := bson.M{}
    
    var results []*Trainer
    
    cur, _ := collection.Find(context.TODO(), filter, options)
    
    for cur.Next(context.TODO()) {
        var elem Trainer
        _ = cur.Decode(&elem)
        fmt.Println("Doc", elem)
        results = append(results, &elem)
    }
    cur.Close(context.TODO())
    fmt.Printf("Found multiple documents (array of pointers): %+v\n", results)
    */
    /*Delete

    CollectionDrop to delete collection
    DeleteMany to delete multiple docs
    DeleteOne to delete one doc

    Example of all bson docs delete in the collection*/
    filterM := bson.M{}
    deleteResult, _ := collection.DeleteMany(context.TODO(), filterM)
    color.Red("Deleted %v documents in the trainers collection\n", deleteResult.DeletedCount)
}

func main() {
    play()
}
